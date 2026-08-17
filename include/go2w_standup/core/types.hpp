// Canonical RobotState / MotorCommand (spec §9).
//
// MotorCommand.tau_ff is only feedforward / WBC / LQR torque. The backend
// must NOT compute Kp*err + Kd*err itself: LowCmd already carries independent
// q/dq/kp/kd/tau fields, so all five fields are passed through verbatim.
#pragma once

#include <Eigen/Dense>

namespace go2w_standup {

// Go2W: 4 legs x [hip, thigh, calf, wheel] (spec §11).
constexpr int kNumJoints = 16;

using JointVector = Eigen::Matrix<double, kNumJoints, 1>;

struct RobotState {
  double time_sec = 0.0;

  Eigen::Quaterniond base_quat = Eigen::Quaterniond::Identity();
  Eigen::Vector3d imu_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_accel = Eigen::Vector3d::Zero();

  JointVector q = JointVector::Zero();
  JointVector dq = JointVector::Zero();
  JointVector tau_est = JointVector::Zero();

  bool valid = false;
};

struct MotorCommand {
  JointVector q_des = JointVector::Zero();
  JointVector dq_des = JointVector::Zero();
  JointVector kp = JointVector::Zero();
  JointVector kd = JointVector::Zero();
  JointVector tau_ff = JointVector::Zero();

  bool enable = false;
};

}  // namespace go2w_standup
