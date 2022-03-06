#include <linear_control.h>
#include <ros/ros.h>
// #define NDEBUG
#include <cassert>
#include <cmath>
#include <iostream>

LinearControl::LinearControl() : mass_(0.49), g_(9.81) {}

void LinearControl::setMass(const double mass) { mass_ = mass; }

void LinearControl::setGravity(const double g) { g_ = g; }

Eigen::Quaterniond ToQuaternion(Eigen::Vector3d Euler) {
  Eigen::Quaterniond q_eigen =
      Eigen::AngleAxisd(Euler[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(Euler[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(Euler[2], Eigen::Vector3d::UnitX());

  double cy = std::cos(Euler[0] * 0.5);
  double sy = std::sin(Euler[0] * 0.5);
  double cp = std::cos(Euler[1] * 0.5);
  double sp = std::sin(Euler[1] * 0.5);
  double cr = std::cos(Euler[2] * 0.5);
  double sr = std::sin(Euler[2] * 0.5);

  Eigen::Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  assert(q_eigen.w() == q.w());
  assert(q_eigen.x() == q.x());
  assert(q_eigen.y() == q.y());
  assert(q_eigen.z() == q.z());

  return q;
}

Eigen::Vector3d ToEuler(Eigen::Quaterniond q) {
  // ERROR
  Eigen::Vector3d ypr_eigen = q.toRotationMatrix().eulerAngles(2, 1, 0);

  Eigen::Vector3d angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  angles[2] = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1)
    angles[1] =
        std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles[1] = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angles[0] = std::atan2(siny_cosp, cosy_cosp);

  // assert(ypr_eigen[0] == angles[0]);
  // assert(ypr_eigen[1] == angles[1]);
  // assert(ypr_eigen[2] == angles[2]);
  return angles;
}

void LinearControl::calculateControl(const Desired_State_t &des,
                                     const Odom_Data_t &odom,
                                     const Imu_Data_t &imu,
                                     Controller_Output_t &u, Gain gain) {
  // Position Control
  Eigen::Vector3d pdd;
  pdd[0] = 0 + gain.Kv0 * (0 - odom.v[0]) + gain.Kp0 * (des.p[0] - odom.p[0]);
  pdd[1] = 0 + gain.Kv1 * (0 - odom.v[1]) + gain.Kp1 * (des.p[1] - odom.p[1]);
  pdd[2] = 0 + gain.Kv2 * (0 - odom.v[2]) + gain.Kp2 * (des.p[2] - odom.p[2]);

  Eigen::Vector3d ypr_state = ToEuler(odom.q);
  Eigen::Vector3d ypr_des = ToEuler(des.q);
  Eigen::Vector3d ypr_c;
  ypr_c[2] =
      1 / g_ *
      (pdd[0] * std::sin(ypr_state[0]) - pdd[1] * std::cos(ypr_state[0]));
  ypr_c[1] =
      1 / g_ *
      (pdd[0] * std::cos(ypr_state[0]) + pdd[1] * std::sin(ypr_state[0]));
  ypr_c[0] = ypr_des[0];

  u.thrust = mass_ * (g_ + pdd[2]);
  u.q = ToQuaternion(ypr_c);

  // SE(3)
  bool use_try_se3_control(true);
  if (use_try_se3_control) {
    Eigen::Vector3d z_B(imu.a[0], imu.a[1], imu.a[2] + g_);
    z_B.normalize();
    Eigen::Vector3d z_W(0, 0, 1);

    Eigen::Vector3d e_p, e_v;
    e_p = odom.p - des.p;
    e_v = odom.v;
    Eigen::Vector3d F_des;
    double K_p(1.0), K_v(1.0);
    F_des = mass_ * g_ * z_W - K_p * e_p - K_v * e_v;
    u.thrust = F_des.dot(z_B);

    Eigen::Vector3d z_B_des = F_des.normalized();
    Eigen::Vector3d x_C_des(std::cos(ypr_des[0]), std::sin(ypr_des[0]), 0);
    Eigen::Vector3d y_B_des = (z_B_des.cross(x_C_des)).normalized();
    Eigen::Vector3d x_B_des = (y_B_des.cross(z_B_des)).normalized();

    Eigen::Matrix<double, 3, 3> R_B_des;
    R_B_des << x_B_des, y_B_des, z_B_des;
    u.q = Eigen::Quaterniond(R_B_des);

    // R_B_des.col(0) = x_B_des;
    // R_B_des.col(1) = y_B_des;
    // R_B_des.col(2) = z_B_des;
    // u.q = R_B_des;
  }
}