#include "linear_control.h"
#include <iostream>
#include <ros/ros.h>


double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{

}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
Eigen::Quaterniond ToQuaternion(Eigen::Vector3d Euler){
  // yaw = Euler[0];
  // pitch = Euler[1];
  // row = Euler[2];
  double cy = cos(Euler[0] * 0.5);
  double sy = sin(Euler[0] * 0.5);
  double cp = cos(Euler[1] * 0.5);
  double sp = sin(Euler[1] * 0.5);
  double cr = cos(Euler[2] * 0.5);
  double sr = sin(Euler[2] * 0.5);
  
  Eigen::Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = cy * sp * cr + sy * cp * sr;
  q.z() = sy * cp * cr - cy * sp * sr;
  
  return q;//code here
}

Eigen::Vector3d ToEuler(Eigen::Quaterniond q){
  Eigen::Vector3d angles;
  //roll
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  angles[2] = std::atan2(sinr_cosp, cosr_cosp);

  //pitch
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if(std::abs(sinp) >= 1)
    angles[1] = std::copysign(M_PI / 2, sinp);
  else
    angles[1] = std::asin(sinp);
    
  //yaw
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.z() * q.z() + q.y() * q.y());
  angles[0] = std::atan2(siny_cosp, cosy_cosp);

  return angles;

  //code here
}
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      Eigen::Vector3d des_acc;
      Eigen::Vector3d Kp,Kv;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;

      
      //对齐imu与动捕坐标轴

      des_acc[0] = 0 + param_.gain.Kv0 * (0 - odom.v[0]) + param_.gain.Kp0 * (des.p[0] - odom.p[0]) ;
      des_acc[1] = 0 + param_.gain.Kv1 * (0 - odom.v[1]) + param_.gain.Kp1 * (des.p[1] - odom.p[1]) ;
      des_acc[2] = 0 + param_.gain.Kv2 * (0 - odom.v[2]) + param_.gain.Kp2 * (des.p[2] - odom.p[2]) + param_.gra;

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
      double roll,pitch,yaw;
      double yaw_odom = fromQuaternion2yaw(odom.q);

      Eigen::Vector3d rpy_state = ToEuler(odom.q);
      Eigen::Vector3d rpy_des = ToEuler(des.q);
      Eigen::Vector3d ypr_;
      ypr_[2] = 1 / param_.gra * (des_acc[0] * std::sin(rpy_state[0]) - des_acc[1] * std::cos(rpy_state[0]));
      ypr_[1] = 1 / param_.gra * (des_acc[0] * std::cos(rpy_state[0]) - des_acc[1] * std::sin(rpy_state[0]));
      ypr_[0] = rpy_state[0];
  /* WRITE YOUR CODE HERE */
      Eigen::Quaterniond q;
      q = ToQuaternion(ypr_);
      u.q = imu.q * odom.q.inverse() * q;
  //used for debug
  debug_msg_.des_p_x = des.p(0);
  debug_msg_.des_p_y = des.p(1);
  debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}

