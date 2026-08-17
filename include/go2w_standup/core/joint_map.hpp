// SDK index <-> canonical joint name mapping (spec §11).
//
// The mapping is explicit in config/robot.yaml; never derive motor indices by
// string sorting. Canonical names always use *_wheel_joint (MJCF convention),
// never *_foot_joint.
#pragma once

#include <array>
#include <string>

#include "go2w_standup/core/types.hpp"

namespace go2w_standup {

enum class Leg { kFR, kFL, kRR, kRL };
enum class JointType { kHip, kThigh, kCalf, kWheel };

struct JointInfo {
  int sdk_index = -1;
  std::string canonical_name;  // e.g. "FR_wheel_joint"
  std::string mjcf_name;       // joint name inside go2w.xml
  std::string axis;            // joint axis from the MJCF, e.g. "0 1 0"
  Leg leg = Leg::kFR;
  JointType type = JointType::kHip;
};

class JointMap {
 public:
  // Loads joint_map + wheel_forward_sign from config/robot.yaml.
  // Throws std::runtime_error on any inconsistency (duplicate index/name,
  // missing entry, unknown leg/type).
  static JointMap LoadFromYaml(const std::string& robot_yaml_path);

  int numJoints() const { return kNumJoints; }

  const JointInfo& at(int sdk_index) const;
  int sdkIndex(const std::string& canonical_name) const;

  Leg leg(int sdk_index) const { return at(sdk_index).leg; }
  JointType type(int sdk_index) const { return at(sdk_index).type; }
  bool isWheel(int sdk_index) const {
    return at(sdk_index).type == JointType::kWheel;
  }

  // +1 if positive motor torque drives the robot forward (+X), else -1.
  double wheelForwardSign(Leg leg) const;
  double wheelForwardSignByIndex(int sdk_index) const {
    return wheelForwardSign(leg(sdk_index));
  }

  // Canonical "forward wheel torque" <-> motor torque (spec §12).
  double wheelMotorTorque(Leg leg, double tau_forward) const {
    return wheelForwardSign(leg) * tau_forward;
  }
  double wheelForwardTorque(int sdk_index, double tau_motor) const {
    return wheelForwardSignByIndex(sdk_index) * tau_motor;
  }

  static const char* LegName(Leg leg);
  static const char* TypeName(JointType type);

 private:
  std::array<JointInfo, kNumJoints> joints_{};
  std::array<double, 4> wheel_forward_sign_{1.0, 1.0, 1.0, 1.0};  // FR FL RR RL
};

}  // namespace go2w_standup
