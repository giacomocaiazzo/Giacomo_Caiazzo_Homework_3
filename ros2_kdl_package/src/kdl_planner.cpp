#include "kdl_planner.h"


//Richiama i costruttori
KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    trajRadius_=0; //set trajRadius_=0 as a flag to decide linear or circular traj in compute_trajectory()  

}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius){

  trajDuration_ = _trajDuration;
  accDuration_ = _accDuration;
  trajInit_ = _trajInit;
  trajRadius_ = _trajRadius;
}

/////////////////////////////////////////////////////////////////

 void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> & _frames,  //Definisce una traiettoria con gli spigoli arrotondati
                                            double _radius, double _eqRadius
                                            ) 
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength()); //Dal punto iniziale (0) e dal punto finale della traiettoria costruisce il profilo di velocità
    traject_ = new KDL::Trajectory_Segment(path_, velpref_); //Costruisce la traiettoria 
}


void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}


void KDLPlanner::trapezoidal_vel(double t, double &s, double &s_dot, double &s_dot_dot){

  double s_c_dot_dot = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_);

  if(t <= accDuration_)
  {
    s = 0.5*s_c_dot_dot*std::pow(t,2);  
    s_dot = s_c_dot_dot*t;
    s_dot_dot = s_c_dot_dot;
  }

  else if(t>accDuration_ && t<=trajDuration_-accDuration_)
  {
    s = s_c_dot_dot*accDuration_*(t-accDuration_/2);
    s_dot = s_c_dot_dot*accDuration_;
    s_dot_dot=0;
  }

  else
  {
    s=1-0.5*s_c_dot_dot*std::pow((trajDuration_-t),2);
    s_dot=s_c_dot_dot*(trajDuration_-t);
    s_dot_dot = -s_c_dot_dot;
  }

}

void KDLPlanner::cubic_polinomial(double t, double &s, double &s_dot, double &s_dot_dot)
{

    double q_i=0;
    double q_f=1;
    double q_i_dot = 0;
    double q_f_dot = 0;
    double a_0=q_i;
    double a_1=q_i_dot;
    double a_2=3/std::pow(trajDuration_,2);
    double a_3=-2/std::pow(trajDuration_,3);

    s=a_3*std::pow(t,3)+a_2*std::pow(t,2)+a_1*t+a_0;
    s_dot=3*a_3*std::pow(t,2)+2*a_2*t+a_1;
    s_dot_dot=6*a_3*t+2*a_2;

}

///////////////////////////////////////////////////////////////////
trajectory_point KDLPlanner::compute_trajectory(double time,std::string trajectory_profile)
{
    if(trajRadius_==0) //compute linear trajectory
    {
        return KDLPlanner::compute_linear_trajectory(time, trajectory_profile);
    }
    else //compute circular trajectory
    {
        return KDLPlanner::compute_circular_trajectory(time, trajectory_profile);
    }

}

/////////////////////////////////////////////////////////////////// da cancellare

trajectory_point KDLPlanner::compute_circular_trajectory(double time,std::string trajectory_profile)
{


    double alpha;
    double s,s_dot,s_dot_dot;

    if(trajectory_profile=="cubic"){
        KDLPlanner::cubic_polinomial(time,s,s_dot,s_dot_dot);
    }
    else if(trajectory_profile=="trap"){
        KDLPlanner::trapezoidal_vel(time,s,s_dot,s_dot_dot);
    }
    else
        throw std::invalid_argument("Invalid velocity profile");

    trajectory_point traj;
    alpha=2*M_PI;

    double x_i=trajInit_[0];
    double y_i=trajInit_[1]+trajRadius_;
    double z_i=trajInit_[2];

    traj.pos[0] = x_i;
    traj.pos[1] = y_i-trajRadius_*cos(alpha*s);
    traj.pos[2] = z_i-trajRadius_*sin(alpha*s);

    traj.vel[0] = 0;
    traj.vel[1] = alpha*trajRadius_*sin(alpha*s)*s_dot;
    traj.vel[2] = -alpha*trajRadius_*cos(alpha*s)*s_dot;

    traj.acc[0] = 0;
    traj.acc[1] = alpha*trajRadius_*(alpha*cos(alpha*s)*std::pow(s_dot,2)+sin(alpha*s)*s_dot_dot);
    traj.acc[2] = -alpha*trajRadius_*(-alpha*sin(alpha*s)*std::pow(s_dot,2)+cos(alpha*s)*s_dot_dot);

    return traj;

}

trajectory_point KDLPlanner::compute_linear_trajectory(double time,std::string trajectory_profile)
{

    Eigen::Vector3d _trajDiff = trajEnd_-trajInit_;
    double s,s_dot,s_dot_dot;
    trajectory_point traj;

    if(trajectory_profile=="cubic")
    {
        KDLPlanner::cubic_polinomial(time,s,s_dot,s_dot_dot);
    }
    else if(trajectory_profile=="trap")
    {
        KDLPlanner::trapezoidal_vel(time,s,s_dot,s_dot_dot);
    }
    else
        throw std::invalid_argument("Invalid velocity profile");


    traj.pos = trajInit_ + (s*_trajDiff);
    traj.vel = s_dot*_trajDiff;
    traj.acc = s_dot_dot*_trajDiff;

    return traj;

}





