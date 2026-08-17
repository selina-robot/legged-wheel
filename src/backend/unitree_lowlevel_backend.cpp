#include "go2w_standup/backend/unitree_lowlevel_backend.hpp"

#include <cmath>
#include <cstdint>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>

namespace go2w_standup {

namespace {

constexpr const char* kTopicLowCmd = "rt/lowcmd";
constexpr const char* kTopicLowState = "rt/lowstate";

// go2 IDL carries 20 motor slots; Go2W actuates the first 16 (spec §11).
constexpr int kIdlNumMotors = 20;

// Motor "stop" sentinels from the official go2 low-level example.
constexpr float kPosStop = 2.146e+9f;
constexpr float kVelStop = 16000.0f;

constexpr uint8_t kServoMode = 0x01;

// CRC32 over the LowCmd message, as required by the firmware (from the
// official unitree_sdk2 go2 low-level example; the sim bridge ignores it but
// real hardware does not).
uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  constexpr uint32_t kPolynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; ++i) {
    xbit = 1u << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; ++bits) {
      if (crc32 & 0x80000000) {
        crc32 <<= 1;
        crc32 ^= kPolynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) crc32 ^= kPolynomial;
      xbit >>= 1;
    }
  }
  return crc32;
}

bool Finite(const JointVector& v) {
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(v[i])) return false;
  }
  return true;
}

}  // namespace

UnitreeLowLevelBackend::UnitreeLowLevelBackend(Config cfg)
    : cfg_(std::move(cfg)) {}

bool UnitreeLowLevelBackend::initialize() {
  unitree::robot::ChannelFactory::Instance()->Init(cfg_.domain_id,
                                                   cfg_.interface);

  lowcmd_pub_.reset(
      new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
          kTopicLowCmd));
  lowcmd_pub_->InitChannel();

  lowstate_sub_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
          kTopicLowState));
  lowstate_sub_->InitChannel(
      [this](const void* msg) { LowStateHandler(msg); }, 1);

  // Start in damping until the first state arrives, so the robot never sees
  // a zero/default command frame.
  Publish(MakeDampingCmd(1.0f));

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(cfg_.init_timeout_sec);
  while (!got_state_.load() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return got_state_.load();
}

void UnitreeLowLevelBackend::LowStateHandler(const void* message) {
  const auto& ls = *static_cast<const unitree_go::msg::dds_::LowState_*>(
      message);

  RobotState s;
  s.time_sec = static_cast<double>(ls.tick()) * 1e-3;

  const auto& imu = ls.imu_state();
  s.base_quat = Eigen::Quaterniond(
      imu.quaternion()[0], imu.quaternion()[1], imu.quaternion()[2],
      imu.quaternion()[3]);
  for (int i = 0; i < 3; ++i) {
    s.imu_gyro[i] = imu.gyroscope()[i];
    s.imu_accel[i] = imu.accelerometer()[i];
  }
  for (int i = 0; i < kNumJoints; ++i) {
    const auto& m = ls.motor_state()[i];
    s.q[i] = m.q();
    s.dq[i] = m.dq();
    s.tau_est[i] = m.tau_est();
  }
  s.valid = Finite(s.q) && Finite(s.dq);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = s;
    last_recv_ = std::chrono::steady_clock::now();
  }
  got_state_.store(true);
}

bool UnitreeLowLevelBackend::read(RobotState& state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!got_state_.load()) return false;
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_recv_).count() >
      cfg_.stale_timeout_sec) {
    return false;
  }
  state = latest_state_;
  return state.valid;
}

bool UnitreeLowLevelBackend::write(const MotorCommand& cmd) {
  if (!cmd.enable) {
    Publish(MakeDampingCmd(1.0f));
    return true;
  }

  unitree_go::msg::dds_::LowCmd_ msg{};
  msg.head()[0] = 0xFE;
  msg.head()[1] = 0xEF;
  msg.level_flag() = 0xFF;
  msg.gpio() = 0;

  for (int i = 0; i < kIdlNumMotors; ++i) {
    auto& m = msg.motor_cmd()[i];
    m.mode() = kServoMode;
    if (i < kNumJoints) {
      // Direct pass-through of the five command fields (spec §9): no
      // Kp*err + Kd*err recomputed here.
      m.q() = static_cast<float>(cmd.q_des[i]);
      m.dq() = static_cast<float>(cmd.dq_des[i]);
      m.kp() = static_cast<float>(cmd.kp[i]);
      m.kd() = static_cast<float>(cmd.kd[i]);
      m.tau() = static_cast<float>(cmd.tau_ff[i]);
    } else {
      m.q() = kPosStop;
      m.dq() = kVelStop;
      m.kp() = 0.0f;
      m.kd() = 0.0f;
      m.tau() = 0.0f;
    }
  }
  Publish(msg);
  return true;
}

void UnitreeLowLevelBackend::emergencyDamping() { Publish(MakeDampingCmd(2.0f)); }

unitree_go::msg::dds_::LowCmd_ UnitreeLowLevelBackend::MakeDampingCmd(
    float kd) {
  unitree_go::msg::dds_::LowCmd_ msg{};
  msg.head()[0] = 0xFE;
  msg.head()[1] = 0xEF;
  msg.level_flag() = 0xFF;
  msg.gpio() = 0;
  for (int i = 0; i < kIdlNumMotors; ++i) {
    auto& m = msg.motor_cmd()[i];
    m.mode() = kServoMode;
    m.q() = kPosStop;
    m.dq() = kVelStop;
    m.kp() = 0.0f;
    m.kd() = kd;
    m.tau() = 0.0f;
  }
  return msg;
}

void UnitreeLowLevelBackend::Publish(
    const unitree_go::msg::dds_::LowCmd_& cmd) {
  if (!lowcmd_pub_) return;
  unitree_go::msg::dds_::LowCmd_ out = cmd;
  out.crc() = Crc32Core(
      reinterpret_cast<uint32_t*>(&out),
      (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  lowcmd_pub_->Write(out);
}

}  // namespace go2w_standup
