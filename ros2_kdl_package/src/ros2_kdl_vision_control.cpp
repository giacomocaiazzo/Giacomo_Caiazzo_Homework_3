// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"


//camera
#include "aruco/markerdetector.h"
#include "aruco/aruco.h"
#include "aruco/posetracker.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_vision_control"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {

            
            // declare cmd_interface parameter (position, velocity, effort)
            declare_parameter("cmd_interface", "effort"); 
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            declare_parameter("task","merge");
            get_parameter("task",task_);
            RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());
            
            //controllo sull'ammissibilità del parametro cmd_interface_
            if (!(cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }
            
            //scelta fra tipo di controllo (look-at-point , positioning , merge) se si scelgie merge non è possibile usare la tecnica di controllo nello spazio operativo (OS)
             while(!(task_== "lap" || task_== "pos" || task_=="merge"))  //joint space inverse dynamic controller o operational space inverse dynamics controller            
            {
                std::cout<<"insert camera choice:"<<std::endl<<"look at point(lap)/positioning(pos)/merge(merge):";
                std::cin>>task_;
                if(!(control_technique_=="lap"||control_technique_=="pos" || task_=="merge"))
                    RCLCPP_INFO(get_logger(),"Selected control technique is not valid!");
            }

            if(cmd_interface_=="effort")
                while(!(control_technique_== "JS" || control_technique_== "OS"))  //joint space inverse dynamic controller o operational space inverse dynamics controller            
                {
                    std::cout<<"insert inverse dynamic controller approach:"<<std::endl<<"joint space(JS)/ operational space(OS):";
                    std::cin>>control_technique_;
                    if(!(control_technique_=="JS"||control_technique_=="OS"))
                        RCLCPP_INFO(get_logger(),"Selected control technique is not valid!");
                }
            else
                control_technique_=="JS"; //unused
                
            double off_roll, off_pitch, off_yaw; 
            double off_x, off_y, off_z;


            std_msgs::msg::Float64MultiArray error_msg;
            error_norm_publisher_ = this->create_publisher<FloatArray>("/cart_err_norm", 10);
            error_norm_publisher_->publish(error_msg); //Publish per aprire il topic

            if(task_!="lap")
            {
                if(task_=="pos")
                {
                    std::cout<<"Insert angular offset in R P Y:";
                    std::cin >> off_roll >> off_pitch >> off_yaw;
                }
                
                std::cout<<"Insert linear offset in X Y Z:";
                std::cin >> off_x >> off_y >> off_z;
            }
            
            orient_offset_= KDL::Rotation::RPY(off_roll, off_pitch, off_yaw);

                        

            trajectory_type_="lin"; //la traiettoria è impostata di default a lineare
            trajectory_profile_="cubic";//il profilo di velocità è impostato di default a cubic

            //inizializzazione dei guadagni
            if(control_technique_=="JS")
            {
                kp_=1;
                kd_=5;
                kpo_=1;  //unused
                kdo_=1;  //unused

            }    
            else if(control_technique_=="OS")
            {
                kp_=20;
                kd_=30;
                kpo_=20;  
                kdo_=30;  

            }    
        
            iteration_ = 0;//this will take account of number of cmd_publisher iterations
            t_ = 0;
            joint_state_available_ = false; 
            aruco_pose_detected_=false;     //è un intero che incrementa il valore ogni volta che viene individuato l'arucoTag (0=non individuato, n= indivudato n-volte)

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) 
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree))
            {
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj);
            joint_torques_.resize(nj); 
            jnt_vel_.resize(nj);
            jnt_acc_.resize(nj);
            jnt_pos_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_)
            {
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

                // Update KDLrobot object
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                KDL::Frame f_T_ee = KDL::Frame::Identity();
                robot_->addEE(f_T_ee);
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // Compute EE frame
                init_cart_pose_ = robot_->getEEFrame();
                init_jnt_pos_.resize(nj);

                init_jnt_pos_.data=robot_->getJntValues();

                // Compute IK in order to know the initial position of the joints
                KDL::JntArray q(nj);
                robot_->getInverseKinematics(init_cart_pose_, q);
 
                // Initialize controller
                KDLController controller_(*robot_);

                // EE's trajectory initial position (just an offset)
                Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));

                // EE's trajectory end position(just opposite y) and wakeup_end_position (a rest position from which the robot will execute the required trajectory)
                Eigen::Vector3d end_position;  



                //Subscriber to aruco marker pose
                //geometry_msgs/msg/PoseStamped
                aruco_mark_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::pose_subscriber, this, std::placeholders::_1));

                // Wait for the joint_state topic
                while(aruco_pose_detected_<1) //finché aruco pose non è individuato attende in questo ciclo while
                {
                    RCLCPP_INFO(this->get_logger(), "No aruco pose received yet! ...");
                    rclcpp::spin_some(node_handle_);
                }


                //final point of the linear trajectory
                end_position<<aruco_frame_bf_.p.data[0]+off_x , aruco_frame_bf_.p.data[1]+off_y , aruco_frame_bf_.p.data[2]+off_z; 

                std::cout<<"END POSITION :"<<end_position(0)<<" "<<end_position(1)<<" "<<end_position(2)<<"\n\n\n\n\n\n\n";
                
                
                // Plan trajectory 
                double traj_duration = 1.5*3, acc_duration = 0.5;
                
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);//linear trajectory (sets the radius=0 as a flag)

                
                p = planner_.compute_trajectory(t_,trajectory_profile_); //viene inizializzato il planner_ usando la traiettoria scelta (default lineare)

                //create the publisher according to cmd_interface_value
                if(cmd_interface_ == "effort") //effort control
                {
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                    



                
                    // Send joint position commands
                    for (long int i = 0; i < joint_torques_.size(); ++i) 
                    {
                        desired_commands_[i] = joint_torques_(i);
                    }       
                    
                }
                else if(cmd_interface_ == "velocity") //velocity control
                { 
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                    {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            
        }

    private:

        void pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
        {
            if((aruco_pose_detected_<1 && task_=="pos")|| task_=="lap" || task_=="merge") // per lap e per merge aggiorna il frame dell'aruco continuamente
            {                                                                                                      // per pos recupera il frame dell'aruco una sola volta
               
                KDL::Frame cameraframe = robot_->getEEFrame();  //recupera camera-frame  

                KDL::Rotation y_rotation = KDL::Rotation::RotY(M_PI);//definizione dell'offset di orientamento
                KDL::Rotation z_rotation = KDL::Rotation::RotZ(M_PI/2); //definizione della matrice di compensazione fra sensor-frame e camera-frame
                KDL::Frame z_rotation_frame;    
                z_rotation_frame.M = z_rotation;

                KDL::Vector aruco_trasl_vec_cf;     //vettore posizione dell'aruco frame nel riferimento della camera
                Eigen::VectorXd aruco_quaternion_cf;//quaternione di orientamento dell'aruco frame nel rif della camera

                aruco_quaternion_cf.resize(4);

                aruco_trasl_vec_cf[0] = pose_msg->pose.position.x;  //recupera i dati dal topic
                aruco_trasl_vec_cf[1] = pose_msg->pose.position.y;
                aruco_trasl_vec_cf[2] = pose_msg->pose.position.z;

                aruco_quaternion_cf[0] = pose_msg->pose.orientation.x;
                aruco_quaternion_cf[1] = pose_msg->pose.orientation.y;
                aruco_quaternion_cf[2] = pose_msg->pose.orientation.z;
                aruco_quaternion_cf[3] = pose_msg->pose.orientation.w;
       
                KDL::Frame aruco_frame_cf_temp; //aruco frame temporaneo utile da compensare e offsettare
                aruco_frame_cf_temp.M = KDL::Rotation::Quaternion(aruco_quaternion_cf[0], aruco_quaternion_cf[1], aruco_quaternion_cf[2], aruco_quaternion_cf[3]);
                aruco_frame_cf_temp.p = aruco_trasl_vec_cf;

                //ora l'aruco frame è espresso nel riferimento del sensore CHE NON COINCIDE CON IL RIFERIMENTO DELLA CAMERA



                aruco_frame_cf_ = z_rotation_frame * aruco_frame_cf_temp; //ho espresso la posizione dell'aruco frame nel camera frame (compensazione)

                aruco_frame_bf_= cameraframe * aruco_frame_cf_; //premoltiplico per la matrice di trasf della camera per riportare l'aruco frame nel world frame
                aruco_frame_bf_.M=aruco_frame_bf_.M*y_rotation; //applco l'offset alle rotazioni
                

                aruco_pose_detected_++;            
            }


        }


        void cmd_publisher()
        {

            iteration_ = iteration_ + 1; //increment iteration_ counter

            // define trajectory
            double trajectory_time = 1.5*3; //the time to complete the requested trajectory
            
            int trajectory_len = 150*3;  
            int loop_rate = trajectory_len / (trajectory_time);
            double dt = 1.0 / loop_rate;
            t_+=dt;

            unsigned int nj = robot_->getNrJnts();
            KDLController controller_(*robot_); 
            // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();

            //###########################################################################

            //implementazione del task look at point
            if(task_=="lap"||task_=="merge")
            {
                Eigen::Vector3d cPo = toEigen(aruco_frame_cf_.p); //converto vettore posizione aruco frame in eigen::
                double cPo_norm=cPo.norm(); //calcolo la norma
                Eigen::Vector3d s = cPo / cPo_norm; //definisco il versore posizione
               
                Eigen::Vector3d sd = Eigen::Vector3d(0, 0, 1);

                //  Compute L(s) matrix
                Eigen::Matrix3d skew_s = skew(s);

                Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

                Eigen::Matrix3d L_lin = -(1.0/cPo_norm) * (I3 - s * s.transpose()); //prime tre colonne della matrice L

                Eigen::Matrix3d R_c = toEigen((robot_->getEEFrame()).M); //recupero la matrice di rotazione da base a camera

                Eigen::Vector3d rot_axis = sd.cross(s); //computo l'asse di rotazione tra versone des e attuale
                double rot_angle =std::acos(sd.dot(s)); //computo l'angolo associato

                lap_frame_.M =(robot_->getEEFrame()).M*(KDL::Rotation::Rot(toKDL(rot_axis), rot_angle)); //costriusco la matrice di rotazione tra i due versori
                lap_frame_.p= robot_->getEEFrame().p; //la posizione desiderata coincide con quella attuale

                if(task_!="merge") //se non sto usando merge calcolo la legge di controllo di lap
                {
                    Eigen::Matrix<double, 3, 6> L;
                    L.setZero();
                    L.block<3,3>(0,0) = L_lin * R_c.transpose();
                    L.block<3,3>(0,3) = skew_s * R_c.transpose();

                    // Compute camera Jacobian
                    KDL::Jacobian camera_jacobian = robot_->getEEJacobian(); 
                    Eigen::MatrixXd J_c(6,7);

                    J_c = camera_jacobian.data;

                    Eigen::MatrixXd LJ_c(3,7);

                    LJ_c = L*J_c;

                    double k = 10;

                    
                    Eigen::MatrixXd N(7,7);
                    N =  Eigen::MatrixXd::Identity(7, 7)-pseudoinverse(LJ_c)*LJ_c;

                    Eigen::VectorXd q_dot_0(7);

                    q_dot_0 = init_jnt_pos_.data-joint_positions_.data; 

                 

                    if(cmd_interface_ == "effort") 
                    {
                        KDL::JntArray ex_joint_velocities; 
                        ex_joint_velocities.resize(nj);
                        ex_joint_velocities.data=jnt_vel_.data;

                        jnt_vel_.data = k*pseudoinverse(LJ_c)*sd+N*q_dot_0; //compute actual joint velocity (lap control law)
                        
                        jnt_pos_.data = jnt_pos_.data + jnt_vel_.data*dt; //compute the discrete integration to obtain position
                        jnt_acc_.data = (jnt_vel_.data-ex_joint_velocities.data)/dt;//compute the discrete derivative to obtain acceleration

                        //velocità desiderata nello spazio operativo:
                        KDL::Twist lap_vel; lap_vel.rot = toKDL(computeOrientationError(toEigen(lap_frame_.M), toEigen(cartpos.M)));
                        KDL::Twist lap_acc;
                        
                        //compute joint_torques
                        if(control_technique_=="JS")
                            joint_torques_=controller_.idCntr(jnt_pos_,jnt_vel_,jnt_acc_,kp_,kd_);
                        else if(control_technique_=="OS")
                            joint_torques_=controller_.idCntr2(lap_frame_, lap_vel,lap_acc,kp_,kpo_,kd_,kdo_);
                    
                    }
                    else if (cmd_interface_ == "velocity")
                    {
                        joint_velocities_.data=k*pseudoinverse(LJ_c)*sd+N*q_dot_0; //compute actual joint velocity (lap control law)
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                    }
                }
            }

            //#######################################################################################



            if(task_=="pos" || task_=="merge") //legge di controllo per il positioning (va eseguita anche per il merge)
            {
                
                // Retrieve the trajectory point
                    
                if(t_<=trajectory_time)// retrive the points of the requested trajectory
                    p = planner_.compute_trajectory(t_,trajectory_profile_);
                else
                {    //don't move
                    if(t_<trajectory_time+dt) //printa solo una volta "end of trajectory"
                         RCLCPP_INFO(this->get_logger(), "End of trajectory execution ...");
                    p = planner_.compute_trajectory(trajectory_time,trajectory_profile_);
                }

                // compute errors //distances between end effector and target position
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));

                Eigen::Vector3d o_error;
                KDL::Frame des_cartpos;

                if(task_=="merge")  //se merge l'orientamento desiderato è fornito da lap
                {
                    o_error = computeOrientationError(toEigen(lap_frame_.M), toEigen(cartpos.M)); //serve per JS non per OS
                    des_cartpos.M = lap_frame_.M; des_cartpos.p = toKDL(p.pos);                  // serve per OS
                }
                
                else //altrimenti è quello dell'aruco (+offset)
                { 
                    o_error = computeOrientationError(toEigen(aruco_frame_bf_.M*orient_offset_), toEigen(cartpos.M)); //serve per JS
                    des_cartpos.M = aruco_frame_bf_.M*orient_offset_; des_cartpos.p = toKDL(p.pos);                  // serve per OS
                
                }

                cart_pos_error_norm_=error.norm();
                cart_o_error_norm_=o_error.norm();
                 //compute desired frames
                //desired frame position
               
                //desired frame velocity
                KDL::Twist des_cartvel2; des_cartvel2.vel=toKDL(p.vel);// des_cartvel2.rot=toKDL(o_error);
                
                //desired frame acceleration
                KDL::Twist des_cartacc; des_cartacc.vel=toKDL(p.acc); //des_cartacc.rot initialized to zero


                
                if(cmd_interface_ == "effort")
                {
                      // Compute differential IK
                     //des_cartvel contains the desired cartesian velocties: << traslation, rotation
                    //includes also an error compensation (increase velocity if the cartesian error(angular or linear) is positive)
                    Vector6d des_cartvel; des_cartvel << p.vel + 10*error, 20*o_error;

                    //save in temporary variable ex_joint_velocities in order to compute discrete derivative
                    KDL::JntArray ex_joint_velocities; 
                    ex_joint_velocities.resize(nj);
                    ex_joint_velocities.data=jnt_vel_.data; //now the last joint velocity (computed in last iteration) is still stored 
                                                                    // in joint_velocities. It's necessary to keep it in a temporary variable

                    jnt_vel_.data = pseudoinverse(robot_->getEEJacobian().data)*des_cartvel; //compute actual joint velocity
                    jnt_pos_.data = jnt_pos_.data + jnt_vel_.data*dt; //compute the discrete integration to obtain position
                    jnt_acc_.data = (jnt_vel_.data-ex_joint_velocities.data)/dt;//compute the discrete derivative to obtain acceleration

                    //compute joint_torques
                    if(control_technique_=="JS")
                        joint_torques_=controller_.idCntr(jnt_pos_,jnt_vel_,jnt_acc_,kp_,kd_);
                    
                    else if(control_technique_=="OS")
                        joint_torques_=controller_.idCntr2(des_cartpos,des_cartvel2,des_cartacc,kp_,kpo_,kd_,kdo_);

                }
                else  //velocity control
                {
                    // Compute differential IK
                    Vector6d des_cartvel; des_cartvel << p.vel + 10*error, 20*o_error;

                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*des_cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                }
            }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ =="effort")
                {

                    // Send effort position commands
                    for (long int i = 0; i < joint_torques_.size(); ++i) 
                    {
                        desired_commands_[i] = joint_torques_(i);
                    }
                 }
                else
                {
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                    {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }   

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                std_msgs::msg::Float64MultiArray error_msg;
                cmd_msg.data = desired_commands_;
                error_msg.data = {cart_pos_error_norm_,cart_o_error_norm_};
                cmdPublisher_->publish(cmd_msg);
                error_norm_publisher_->publish(error_msg);


                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // st d::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            
/*            else
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                {
                    desired_commands_[i] = 0;
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
  */
        }


         


        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg)
        {

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++)
            {
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::Publisher<FloatArray>::SharedPtr error_norm_publisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_accelerations_;
        KDL::JntArray jnt_pos_;
        KDL::JntArray jnt_vel_;
        KDL::JntArray jnt_acc_;
        Eigen::VectorXd joint_torques_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        KDLPlanner wakeup_planner_;
        int iteration_;
        bool joint_state_available_;
        double t_;
        KDL::JntArray init_jnt_pos_;
        
        std::string cmd_interface_, trajectory_type_, trajectory_profile_, control_technique_, task_;
        
        KDL::Frame init_cart_pose_;
        double kp_,kd_,kpo_,kdo_;
        trajectory_point p;


        //camera
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_mark_pose_sub_;

        KDL::Frame aruco_frame_bf_;
        KDL::Frame aruco_frame_cf_;
        int aruco_pose_detected_;

        KDL::Frame lap_frame_;
        KDL::Rotation orient_offset_; //offset di orientamento

        double cart_pos_error_norm_;
        double cart_o_error_norm_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
