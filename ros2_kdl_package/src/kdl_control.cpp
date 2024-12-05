#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis();// + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr2(KDL::Frame &_desPos,
                                       KDL::Twist &_desVel,
                                       KDL::Twist &_desAcc,
                                       double _Kpp, double _Kpo,
                                       double _Kdp, double _Kdo)
{
    //define Gain Matrices

    Matrix6d KP_matrix, KD_matrix;
    KP_matrix.setZero();
    KD_matrix.setZero();
    
    for(int i=0; i<3; ++i)
        KP_matrix(i,i)=_Kpp;
    for(int i=3; i<6; ++i)
        KP_matrix(i,i)=_Kpo;
    
    for(int i=0; i<3; ++i)
        KD_matrix(i,i)=_Kdp;
    for(int i=3; i<6; ++i)
        KD_matrix(i,i)=_Kdo;
        
    
/*
    for(int i=0;i<6; ++i)
    {
        for(int j=0; j<6; ++j)
            std::cout<<KP_matrix.coeff(i,j);
        std::cout<<std::endl;
    }

    for(int i=0;i<6; ++i)
    {
        for(int j=0; j<6; ++j)
            std::cout<<KD_matrix.coeff(i,j);
        std::cout<<std::endl;
    }
*/    

    //allocate vector for to-be-computed output torques
    Eigen::VectorXd output_torques, y;

    //retrieve current EE frame and twist
    KDL::Frame current_frame = robot_->getEEFrame();
    KDL::Twist current_twist = robot_->getEEVelocity();

    //retrive and convert jacobian to a matrixXd
    Eigen::MatrixXd Jacobian; 
    Jacobian=robot_->getEEJacobian().data;

    //retrive JacDotqDot
    Eigen::VectorXd JacDotqDot;
    JacDotqDot = robot_->getEEJacDotqDot();

    //convert _desAcc to a VectorXd type
    Eigen::VectorXd desAcc_vec;
    desAcc_vec.resize(6);
    desAcc_vec<< _desAcc.vel.x(), _desAcc.vel.y(), _desAcc.vel.z(),
                 _desAcc.rot.x(), _desAcc.rot.y(), _desAcc.rot.z();

    //allocate vectors for position and velocity errors and compute
    Vector6d position_error, velocity_error; 

    computeErrors(_desPos, current_frame, _desVel, current_twist,position_error, velocity_error);

    y=pseudoinverse(Jacobian)*(desAcc_vec+KD_matrix*velocity_error+KP_matrix*position_error-JacDotqDot);

    output_torques=robot_->getJsim()*y + robot_->getCoriolis();// + robot_->getGravity();
    
    return output_torques;
}

