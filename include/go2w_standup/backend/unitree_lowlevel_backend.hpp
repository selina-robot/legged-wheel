// Unitree LowCmd/LowState DDS backend (spec §10). All DDS traffic for the
// low-level interface lives in this class; nothing outside it touches
// ChannelPublisher/ChannelSubscriber for rt/lowcmd / rt/lowstate.
//
// Go2W uses the go2 IDL: motor_cmd/motor_state are std::array<..., 20>, of
// which indices 0..15 are the 16 actuated joints in SDK order (12 leg motors
// FR/FL/RR/RL x hip/thigh/calf, then 4 wheels FR/FL/RR/RL). Confirmed against
// unitree_mujoco's Go2Bridge (ctrl[i] <-> actuator i, i < mj_model->nu = 16).
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "go2w_standup/backend/robot_backend.hpp"

namespace go2w_standup {

class UnitreeLowLevelBackend : public RobotBackend {
 public:
  struct Config {
    int domain_id = 1;      // 1 = MuJoCo sim, 0 = future hardware
    std::string interface = "lo";
    double init_timeout_sec = 5.0;    // wait for first LowState
    double stale_timeout_sec = 0.1;   // read() fails if state older than this
  };

  explicit UnitreeLowLevelBackend(Config cfg);
  ~UnitreeLowLevelBackend() override = default;

  bool initialize() override;
  bool read(RobotState& state) override;
  bool write(const MotorCommand& cmd) override;
  void emergencyDamping() override;

 private:
  void LowStateHandler(const void* message);
  void Publish(const unitree_go::msg::dds_::LowCmd_& cmd);
  // Small damping on all motors: kp=0, kd=damping, tau=0, position stop.
  static unitree_go::msg::dds_::LowCmd_ MakeDampingCmd(float kd);

  Config cfg_;

  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>
      lowcmd_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      lowstate_sub_;

  std::mutex state_mutex_;
  RobotState latest_state_;
  std::chrono::steady_clock::time_point last_recv_;
  std::atomic<bool> got_state_{false};
};

}  // namespace go2w_standup
