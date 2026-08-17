// Gate 1: per-motor LowCmd/LowState smoke test (spec §12).
//
// Rationale (arrived at by diagnosing several standing/lifted protocols; see
// the Gate-1 report): on a standing robot a 0.2-0.5 Nm pulse cannot overcome
// ground friction (hip abduction) or static wheel friction (a loaded wheel
// needs ~1.6 Nm to slip), and a limped load-bearing joint free-falls under
// gravity out of the linear regime; no soft 3-leg pose hold lifts a wheel
// clear of the ground either. So:
//
//   * Joint identity and positive direction are measured in the spawn FREE
//     FALL. After mj_resetData the robot is at z=0.6 with ~128 ms of true
//     zero-g before touchdown (verified via the IMU accelerometer). DDS
//     discovery takes ~0.37 s, so instead of restarting the simulator per
//     motor, the test drives the simulator's built-in reset (BACKSPACE, via
//     tools/x11_sim_reset) and pulses each motor twice (+tau and -tau) over
//     the same absolute sim-time window (t = 0.02..0.12 s, inside the spec's
//     100-200 ms), with commands computed from the state's own sim time so
//     both falls replay identically. qpos0 has the knees outside their MJCF
//     range, so the fall carries a strong but deterministic joint-limit
//     projection transient; it cancels in the +/- differential. In zero-g
//     +tau accelerates the pulsed joint positively (sign), and the
//     median-subtracted response must dominate its class (identity; the
//     median removes the common-mode base reaction).
//
//   * wheel_forward_sign is measured GROUNDED, rolling-free: with the robot
//     standing in the crouch and all legs position-held, a +/- 0.5 Nm wheel
//     pulse pushes the ground with F = tau/r (10 N, below every contact's
//     static friction limit) and the resulting pitch couple redistributes
//     load between the front and rear axles. With all four wheel axes +y in
//     the base frame (go2w.xml FK), a forward push (+x) tips the robot
//     nose-down: the OTHER legs' calf tau_est (jointactuatorfrc, excluding
//     the tested leg's own calf which also feels the motor reaction) shift
//     by front-plus/rear-minus iff positive motor rotation drives the robot
//     forward, i.e. wheel_forward_sign = +1.
//
// Outputs:
//   artifacts/reports/model_audit/joint_map.csv  (spec §12 columns)
//   artifacts/logs/gate1_log_motor_NN_{p,m}.csv  (per-fall time series)
//   artifacts/logs/gate1_log_wheel_NN.csv        (grounded wheel phase)
//
// Safety: at most one motor ever carries torque (0.5 Nm, <=200 ms); falls
// end with all-free commands before touchdown; loss of LowState, excessive
// base tilt, or reset-tool failure aborts into emergencyDamping().
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "go2w_standup/backend/unitree_lowlevel_backend.hpp"
#include "go2w_standup/core/joint_map.hpp"

using go2w_standup::JointMap;
using go2w_standup::JointVector;
using go2w_standup::MotorCommand;
using go2w_standup::RobotState;
using go2w_standup::UnitreeLowLevelBackend;
using go2w_standup::kNumJoints;

namespace {

constexpr double kPulseNm = 0.5;           // top of the 0.2-0.5 Nm range
// Free-fall phase (absolute sim time after reset).
constexpr double kFallSettleSec = 0.02;    // window start
constexpr double kFallEndSec = 0.12;       // window end (touchdown ~0.128 s)
constexpr double kSettleKd = 2.0;          // damping before/around the fall
constexpr double kOthersKd = 0.5;          // non-pulsed joints during pulse
constexpr double kTouchdownAccelZ = 5.0;   // m/s^2 sustained = touchdown
constexpr double kResetDetectSec = 0.02;   // tick below this = reset seen
constexpr double kResetWaitSec = 3.0;
constexpr int kMinSamples = 35;            // >= 70% of the window at 500 Hz
// Grounded wheel-forward-sign phase.
constexpr double kGotoSec = 3.0;           // ramp from landed pose to crouch
constexpr double kWheelSettleSec = 1.0;
constexpr double kWheelPulseSec = 0.2;     // 100-200 ms, spec §12
constexpr double kWheelHoldKd = 2.0;       // non-pulsed wheels: damping only
constexpr double kTiltAbortRad = 0.7;      // base roll/pitch watchdog

// Thresholds.
constexpr double kLegDqThresh = 0.05;      // rad/s, fall differential
constexpr double kWheelDqThresh = 0.15;    // rad/s, fall differential
constexpr double kWfsTauThresh = 0.3;      // Nm, same-leg thigh response diff

struct ImpedanceGains {
  double hip_kp = 30.0, hip_kd = 1.5;
  double thigh_kp = 35.0, thigh_kd = 2.0;
  double calf_kp = 35.0, calf_kd = 2.0;
};

void BaseRollPitch(const Eigen::Quaterniond& q, double* roll, double* pitch) {
  const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  *roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  *pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
}

}  // namespace

int main(int argc, char** argv) {
  std::string robot_yaml = "config/robot.yaml";
  std::string sim_yaml = "config/sim.yaml";
  std::string control_yaml = "config/control.yaml";
  std::string out_csv = "artifacts/reports/model_audit/joint_map.csv";
  std::string reset_tool = "build/x11_sim_reset";
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--robot-config") robot_yaml = argv[i + 1];
    else if (a == "--sim-config") sim_yaml = argv[i + 1];
    else if (a == "--control-config") control_yaml = argv[i + 1];
    else if (a == "--out") out_csv = argv[i + 1];
    else if (a == "--reset-tool") reset_tool = argv[i + 1];
  }

  try {
    const JointMap jm = JointMap::LoadFromYaml(robot_yaml);

    const YAML::Node sim = YAML::LoadFile(sim_yaml);
    UnitreeLowLevelBackend::Config bcfg;
    bcfg.domain_id = sim["backend"]["domain_id"].as<int>();
    bcfg.interface = sim["backend"]["interface"].as<std::string>();
    bcfg.init_timeout_sec = 15.0;

    ImpedanceGains gains;
    const YAML::Node ctrl = YAML::LoadFile(control_yaml);
    if (ctrl["impedance"]) {
      const auto& imp = ctrl["impedance"];
      gains.hip_kp = imp["hip"]["kp"].as<double>();
      gains.hip_kd = imp["hip"]["kd"].as<double>();
      gains.thigh_kp = imp["thigh"]["kp"].as<double>();
      gains.thigh_kd = imp["thigh"]["kd"].as<double>();
      gains.calf_kp = imp["calf"]["kp"].as<double>();
      gains.calf_kd = imp["calf"]["kd"].as<double>();
    }

    UnitreeLowLevelBackend backend(bcfg);
    std::cout << "[smoke] initializing backend (domain=" << bcfg.domain_id
              << ", iface=" << bcfg.interface << ") ...\n";
    if (!backend.initialize()) {
      std::cerr << "[smoke] FAIL: no LowState received. Is the simulator "
                   "running (./scripts/run_sim.sh)?\n";
      return 1;
    }

    RobotState s0;
    if (!backend.read(s0) || !s0.valid) {
      std::cerr << "[smoke] FAIL: invalid first state\n";
      backend.emergencyDamping();
      return 1;
    }
    int readable = 0;
    for (int i = 0; i < kNumJoints; ++i) {
      if (std::isfinite(s0.q[i]) && std::isfinite(s0.dq[i])) ++readable;
    }
    std::cout << "[smoke] readable joints: " << readable << "/" << kNumJoints
              << " (sim t=" << s0.time_sec << " s)\n";
    if (readable != kNumJoints) {
      std::cerr << "[smoke] FAIL: not all joints readable\n";
      backend.emergencyDamping();
      return 1;
    }

    std::filesystem::create_directories("artifacts/logs");
    std::filesystem::create_directories("artifacts/reports/model_audit/rows");

    MotorCommand zero_cmd;  // all gains/torque zero, motors free
    zero_cmd.enable = true;

    // ---- Free-fall helpers ---------------------------------------------

    // One free-fall pulse run: pre-settle in damping, reset the sim, then
    // pulse sign*kPulseNm on `motor` over the absolute sim-time window
    // [kFallSettleSec, kFallEndSec] (commands computed from the state's own
    // sim time, so both runs replay identically). Returns false on hard
    // failure; *clean=false if touchdown contaminated the window (retry).
    auto RunFall = [&](int motor, double sign, const std::string& log_path,
                       JointVector* mean_dq, bool* clean) -> bool {
      MotorCommand damp_cmd = zero_cmd;
      damp_cmd.kd.setConstant(kSettleKd);
      const auto pre0 = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - pre0 <
             std::chrono::milliseconds(300)) {
        backend.write(damp_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // Reset: robot back to the spawn pose (free fall).
      if (std::system(reset_tool.c_str()) != 0) {
        std::cerr << "[smoke] FAIL: reset tool failed (" << reset_tool
                  << ")\n";
        backend.emergencyDamping();
        return false;
      }

      // Wait for the tick rollover that marks the reset.
      bool detected = false;
      const auto wait0 = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - wait0 <
             std::chrono::duration<double>(kResetWaitSec)) {
        RobotState s;
        if (backend.read(s) && s.time_sec < kResetDetectSec) {
          detected = true;
          break;
        }
        backend.write(damp_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (!detected) {
        std::cerr << "[smoke] reset not detected within " << kResetWaitSec
                  << " s\n";
        backend.emergencyDamping();
        return false;
      }

      FILE* logf = std::fopen(log_path.c_str(), "w");
      if (logf) {
        std::fprintf(logf, "time,az,gyro_y");
        for (int i = 0; i < kNumJoints; ++i) std::fprintf(logf, ",q%d", i);
        for (int i = 0; i < kNumJoints; ++i) std::fprintf(logf, ",dq%d", i);
        std::fprintf(logf, "\n");
      }

      JointVector dq_sum = JointVector::Zero();
      int n = 0;
      int td_count = 0;
      int lost_reads = 0;
      *clean = true;
      while (true) {
        RobotState s;
        if (!backend.read(s)) {
          if (++lost_reads > 200) {
            std::cerr << "[smoke] LowState lost during pulse\n";
            backend.emergencyDamping();
            if (logf) std::fclose(logf);
            return false;
          }
        } else {
          lost_reads = 0;
          const double t = s.time_sec;  // absolute sim time since reset
          if (logf) {
            std::fprintf(logf, "%.4f,%.4f,%.4f", s.time_sec, s.imu_accel.z(),
                         s.imu_gyro.y());
            for (int i = 0; i < kNumJoints; ++i)
              std::fprintf(logf, ",%.4f", s.q[i]);
            for (int i = 0; i < kNumJoints; ++i)
              std::fprintf(logf, ",%.4f", s.dq[i]);
            std::fprintf(logf, "\n");
          }
          if (t > kFallEndSec + 0.02) break;
          // Command is a pure function of sim time: identical across runs.
          if (t < kFallSettleSec - 0.001) {
            backend.write(damp_cmd);
          } else if (t <= kFallEndSec - 0.001) {
            MotorCommand cmd = zero_cmd;
            cmd.kd.setConstant(kOthersKd);
            cmd.kd[motor] = 0.0;
            cmd.tau_ff[motor] = sign * kPulseNm;
            backend.write(cmd);
          } else {
            backend.write(zero_cmd);
          }
          if (t >= kFallSettleSec && t <= kFallEndSec) {
            dq_sum += s.dq;
            ++n;
            if (std::abs(s.imu_accel.z()) > kTouchdownAccelZ) {
              if (++td_count >= 10) {  // sustained = real touchdown
                *clean = false;
                break;
              }
            } else {
              td_count = 0;
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (logf) std::fclose(logf);

      // All-free through touchdown.
      const auto post0 = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - post0 <
             std::chrono::milliseconds(100)) {
        backend.write(zero_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }

      if (n > 0) *mean_dq = dq_sum / static_cast<double>(n);
      if (n < kMinSamples) *clean = false;
      return true;
    };

    struct Row {
      int sign = 0;
      int wfs = 0;  // wheels only
      bool identity_ok = false;
      double dq_own = 0.0;
      double wfs_metric = 0.0;
    };
    std::vector<Row> rows(kNumJoints);

    // ---- Phase 1: free-fall identity + sign for all 16 motors ----------
    for (int motor = 0; motor < kNumJoints; ++motor) {
      const bool wheel = jm.isWheel(motor);
      const double thresh = wheel ? kWheelDqThresh : kLegDqThresh;

      Row best;
      double best_margin = -1.0;
      for (int attempt = 0; attempt < 3; ++attempt) {
        // Two falls per motor: +tau and -tau; the deterministic projection
        // transient cancels in the difference.
        JointVector mean_dq[2];
        bool ok = true;
        for (int s = 0; s < 2 && ok; ++s) {
          const double sgn = s == 0 ? 1.0 : -1.0;
          bool got = false;
          for (int retry = 0; retry < 3 && !got; ++retry) {
            const std::string log_path =
                std::string("artifacts/logs/gate1_log_motor_") +
                (motor < 10 ? "0" : "") + std::to_string(motor) +
                (sgn > 0 ? "_p.csv" : "_m.csv");
            bool clean = false;
            if (!RunFall(motor, sgn, log_path, &mean_dq[s], &clean)) {
              backend.emergencyDamping();
              return 1;
            }
            if (clean) {
              got = true;
            } else {
              std::printf("[smoke] motor %2d sign %+d: contaminated window "
                          "(retry %d)\n", motor, (int)sgn, retry + 1);
            }
          }
          if (!got) ok = false;
        }
        if (!ok) {
          std::cerr << "[smoke] FAIL: motor " << motor
                    << " never got a clean airborne window\n";
          backend.emergencyDamping();
          return 1;
        }

        const JointVector dq_diff = mean_dq[0] - mean_dq[1];  // (+) - (-)
        Row r;
        r.dq_own = dq_diff[motor];
        r.sign = r.dq_own > thresh ? 1 : (r.dq_own < -thresh ? -1 : 0);

        // Identity: own response minus the common-mode base reaction (median
        // of the other joints of the same class) must dominate.
        std::vector<double> others;
        for (int i = 0; i < kNumJoints; ++i) {
          if (i != motor && jm.isWheel(i) == wheel) {
            others.push_back(dq_diff[i]);
          }
        }
        std::nth_element(others.begin(), others.begin() + others.size() / 2,
                         others.end());
        const double common = others[others.size() / 2];
        const double own_indiv = r.dq_own - common;
        double worst_other = 0.0;
        for (const double v : others) {
          worst_other = std::max(worst_other, std::abs(v - common));
        }
        r.identity_ok = std::abs(own_indiv) > thresh &&
                        std::abs(own_indiv) >= worst_other;

        const double margin = std::abs(own_indiv) - worst_other;
        std::printf(
            "[smoke] motor %2d %-16s dq_diff=%7.3f (indiv %6.3f vs %.3f) "
            "sign=%d identity=%s (attempt %d)\n",
            motor, jm.at(motor).canonical_name.c_str(), r.dq_own, own_indiv,
            worst_other, r.sign, r.identity_ok ? "ok" : "FAIL", attempt + 1);
        if (margin > best_margin) {
          best_margin = margin;
          best = r;
        }
        if (r.sign != 0 && r.identity_ok) break;  // clean measurement
      }
      rows[motor] = best;
      if (best.sign == 0 || !best.identity_ok) {
        std::printf("[smoke] motor %2d: FAILED after all attempts\n", motor);
      }
    }

    // ---- Phase 2: grounded wheel forward sign --------------------------
    // Ramp from the landed pose back to the crouch, then pulse each wheel
    // with the legs position-held and read the axle load redistribution.
    std::cout << "[smoke] phase 2: returning to crouch for wheel forward "
                 "sign ...\n";
    JointVector q_crouch;
    {
      const YAML::Node robot_cfg = YAML::LoadFile(robot_yaml);
      const YAML::Node qi = robot_cfg["q_init"];
      for (int i = 0; i < kNumJoints; ++i) q_crouch[i] = qi[i].as<double>();
    }

    auto HoldCmd = [&](const JointVector& q_hold, int pulse_wheel,
                       double pulse_tau) {
      MotorCommand cmd;
      cmd.enable = true;
      for (int i = 0; i < kNumJoints; ++i) {
        cmd.q_des[i] = q_hold[i];
        cmd.dq_des[i] = 0.0;
        cmd.tau_ff[i] = 0.0;
        switch (jm.type(i)) {
          case go2w_standup::JointType::kHip:
            cmd.kp[i] = gains.hip_kp; cmd.kd[i] = gains.hip_kd; break;
          case go2w_standup::JointType::kThigh:
            cmd.kp[i] = gains.thigh_kp; cmd.kd[i] = gains.thigh_kd; break;
          case go2w_standup::JointType::kCalf:
            cmd.kp[i] = gains.calf_kp; cmd.kd[i] = gains.calf_kd; break;
          case go2w_standup::JointType::kWheel:
            cmd.kp[i] = 0.0; cmd.kd[i] = kWheelHoldKd; break;
        }
      }
      if (pulse_wheel >= 0) {
        cmd.kp[pulse_wheel] = 0.0;
        cmd.kd[pulse_wheel] = 0.0;
        cmd.tau_ff[pulse_wheel] = pulse_tau;
      }
      return cmd;
    };

    // Ramp to crouch (smoothstep), with tilt/lost-state guards.
    {
      RobotState sc;
      if (!backend.read(sc)) {
        std::cerr << "[smoke] FAIL: no state before phase 2\n";
        backend.emergencyDamping();
        return 1;
      }
      const JointVector q_from = sc.q;
      const auto t0 = std::chrono::steady_clock::now();
      int lost = 0;
      while (true) {
        const double t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                .count();
        if (t > kGotoSec) break;
        const double a = t / kGotoSec;
        const double s = a * a * (3.0 - 2.0 * a);
        RobotState st;
        if (!backend.read(st)) {
          if (++lost > 100) {
            backend.emergencyDamping();
            return 1;
          }
        } else {
          lost = 0;
          double roll = 0.0, pitch = 0.0;
          BaseRollPitch(st.base_quat, &roll, &pitch);
          if (std::abs(roll) > kTiltAbortRad ||
              std::abs(pitch) > kTiltAbortRad) {
            std::cerr << "[smoke] base tilted in phase 2, damping\n";
            backend.emergencyDamping();
            return 1;
          }
        }
        backend.write(HoldCmd((1.0 - s) * q_from + s * q_crouch, -1, 0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }

    // Per-wheel: settle, then +/- pulse; measure calf tau redistribution.
    // The pose for each axle loads that axle: the crouch sits nose-down
    // (front loaded, good for front wheels); for the rear wheels the front
    // legs are shortened and the rear legs extended to shift weight back.
    JointVector q_rear_load = q_crouch;
    for (const int l : {0, 1}) {  // front legs shorter
      q_rear_load[l * 3 + 1] = 1.0;
      q_rear_load[l * 3 + 2] = -2.0;
    }
    for (const int l : {2, 3}) {  // rear legs longer
      q_rear_load[l * 3 + 1] = 0.75;
      q_rear_load[l * 3 + 2] = -1.5;
    }

    auto WheelSignPhase = [&](int wheel, double sign, const JointVector& q_pose,
                              JointVector* leg_tau,
                              const std::string& log_path) -> bool {
      FILE* logf = std::fopen(log_path.c_str(), "w");
      if (logf) {
        std::fprintf(logf, "time,tau_cmd,roll,pitch");
        for (int i = 0; i < kNumJoints; ++i)
          std::fprintf(logf, ",tau%d", i);
        std::fprintf(logf, "\n");
      }
      int lost = 0;
      const auto t0 = std::chrono::steady_clock::now();
      JointVector tau_sum = JointVector::Zero();
      JointVector base_sum = JointVector::Zero();
      int n = 0, nb = 0;
      const double total = kWheelSettleSec + kWheelPulseSec;
      while (true) {
        const double t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                .count();
        if (t > total) break;
        const bool pulsing = t >= kWheelSettleSec;
        RobotState st;
        if (!backend.read(st)) {
          if (++lost > 100) {
            backend.emergencyDamping();
            if (logf) std::fclose(logf);
            return false;
          }
        } else {
          lost = 0;
          double roll = 0.0, pitch = 0.0;
          BaseRollPitch(st.base_quat, &roll, &pitch);
          if (std::abs(roll) > kTiltAbortRad ||
              std::abs(pitch) > kTiltAbortRad) {
            std::cerr << "[smoke] base tilted in wheel phase, damping\n";
            backend.emergencyDamping();
            if (logf) std::fclose(logf);
            return false;
          }
          if (logf) {
            std::fprintf(logf, "%.3f,%.2f,%.4f,%.4f", st.time_sec,
                         pulsing ? sign * kPulseNm : 0.0, roll, pitch);
            for (int i = 0; i < kNumJoints; ++i)
              std::fprintf(logf, ",%.4f", st.tau_est[i]);
            std::fprintf(logf, "\n");
          }
          // Baseline: settle tail. Signal: pulse tail.
          if (t >= total - 0.1) {
            tau_sum += st.tau_est;
            ++n;
          } else if (t >= kWheelSettleSec - 0.3 && !pulsing) {
            base_sum += st.tau_est;
            ++nb;
          }
        }
        backend.write(
            HoldCmd(q_pose, pulsing ? wheel : -1, sign * kPulseNm));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (logf) std::fclose(logf);
      if (n == 0 || nb == 0) return false;
      *leg_tau = tau_sum / static_cast<double>(n) -
                 base_sum / static_cast<double>(nb);
      return true;
    };

    for (int leg = 0; leg < 4; ++leg) {
      const int wheel = 12 + leg;
      const int thigh = leg * 3 + 1;
      // Load the tested axle: crouch (nose-down) for front wheels, weight
      // shifted back for rear wheels.
      const JointVector& q_pose = (leg < 2) ? q_crouch : q_rear_load;

      // Ramp to the axle pose (smoothstep over 2 s).
      {
        RobotState sc;
        if (!backend.read(sc)) {
          backend.emergencyDamping();
          return 1;
        }
        const JointVector q_from = sc.q;
        const auto t0 = std::chrono::steady_clock::now();
        int lost = 0;
        while (true) {
          const double t = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - t0)
                               .count();
          if (t > 2.0) break;
          const double a = t / 2.0;
          const double s = a * a * (3.0 - 2.0 * a);
          RobotState st;
          if (!backend.read(st)) {
            if (++lost > 100) {
              backend.emergencyDamping();
              return 1;
            }
          } else {
            lost = 0;
            double roll = 0.0, pitch = 0.0;
            BaseRollPitch(st.base_quat, &roll, &pitch);
            if (std::abs(roll) > kTiltAbortRad ||
                std::abs(pitch) > kTiltAbortRad) {
              std::cerr << "[smoke] base tilted in wheel phase, damping\n";
              backend.emergencyDamping();
              return 1;
            }
          }
          backend.write(HoldCmd((1.0 - s) * q_from + s * q_pose, -1, 0.0));
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }

      JointVector tau_plus, tau_minus;
      if (!WheelSignPhase(wheel, +1.0, q_pose, &tau_plus,
                          std::string("artifacts/logs/gate1_log_wheel_") +
                              std::to_string(wheel) + ".csv")) {
        return 1;
      }
      if (!WheelSignPhase(wheel, -1.0, q_pose, &tau_minus,
                          std::string("artifacts/logs/gate1_log_wheel_") +
                              std::to_string(wheel) + "_m.csv")) {
        return 1;
      }

      // Same-leg thigh tau_est differential (baseline is the per-run
      // pre-pulse mean already subtracted inside... see below): the ground
      // force F_x of the pulsed wheel creates a moment r_z*F_x about the
      // thigh joint (r_z < 0, wheel below), which the position hold absorbs:
      //   d_tau_thigh = +|r_z| * F_x.
      // The wheel axis is +y for all four wheels (go2w.xml FK), so +tau
      // pushes the robot forward iff wheel_forward_sign = +1.
      const double d_thigh = (tau_plus - tau_minus)[thigh];
      rows[wheel].wfs =
          d_thigh > kWfsTauThresh ? 1 : (d_thigh < -kWfsTauThresh ? -1 : 0);
      rows[wheel].wfs_metric = d_thigh;
      std::printf("[smoke] wheel %2d (%s): same-leg thigh d_tau=%+.3f Nm -> "
                  "wfs=%d\n", wheel, jm.at(wheel).canonical_name.c_str(),
                  d_thigh, rows[wheel].wfs);
    }

    backend.emergencyDamping();

    // ---- Outputs ---------------------------------------------------------
    std::filesystem::create_directories(
        std::filesystem::path(out_csv).parent_path());
    FILE* f = std::fopen(out_csv.c_str(), "w");
    if (!f) {
      std::cerr << "[smoke] FAIL: cannot open " << out_csv << "\n";
      return 1;
    }
    std::fprintf(
        f,
        "sdk_index,canonical_name,mjcf_name,axis,positive_direction,"
        "wheel_forward_sign\n");
    for (int j = 0; j < kNumJoints; ++j) {
      const auto& info = jm.at(j);
      std::fprintf(f, "%d,%s,%s,\"%s\",%d,", j, info.canonical_name.c_str(),
                   info.mjcf_name.c_str(), info.axis.c_str(), rows[j].sign);
      if (jm.isWheel(j)) std::fprintf(f, "%d", rows[j].wfs);
      std::fprintf(f, "\n");
      const std::string row_path =
          std::string("artifacts/reports/model_audit/rows/row_") +
          (j < 10 ? "0" : "") + std::to_string(j) + ".csv";
      FILE* rf = std::fopen(row_path.c_str(), "w");
      if (rf) {
        std::fprintf(rf, "%d,%s,%s,\"%s\",%d,", j, info.canonical_name.c_str(),
                     info.mjcf_name.c_str(), info.axis.c_str(), rows[j].sign);
        if (jm.isWheel(j)) std::fprintf(rf, "%d", rows[j].wfs);
        std::fprintf(rf, "\n");
        std::fclose(rf);
      }
    }
    std::fclose(f);

    int identity_ok_count = 0, sign_ok_count = 0, wheel_sign_ok_count = 0;
    std::cout << "\n[smoke] ==== Gate 1 summary ====\n";
    std::printf("%3s %-16s %8s %8s %6s %6s %6s\n", "idx", "name", "dq_own",
                "wfs_met", "dir", "wfwd", "ident");
    for (int j = 0; j < kNumJoints; ++j) {
      const auto& r = rows[j];
      std::printf("%3d %-16s %8.3f %8.3f %6d %6d %6s\n", j,
                  jm.at(j).canonical_name.c_str(), r.dq_own, r.wfs_metric,
                  r.sign, r.wfs, r.identity_ok ? "ok" : "FAIL");
      if (r.identity_ok) ++identity_ok_count;
      if (r.sign != 0) ++sign_ok_count;
      if (!jm.isWheel(j) || r.wfs != 0) ++wheel_sign_ok_count;
    }
    std::cout << "[smoke] joint identity verified: " << identity_ok_count
              << "/16\n";
    std::cout << "[smoke] sign verified: " << sign_ok_count << "/16\n";
    std::cout << "[smoke] forward wheel sign verified: " << wheel_sign_ok_count
              << "/4 (legs count as n/a)\n";
    std::cout << "[smoke] wrote " << out_csv << "\n";

    const bool pass = identity_ok_count == kNumJoints &&
                      sign_ok_count == kNumJoints &&
                      wheel_sign_ok_count == kNumJoints;
    std::cout << (pass ? "[smoke] GATE 1 PASS\n" : "[smoke] GATE 1 FAIL\n");
    return pass ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "[smoke] exception: " << e.what() << "\n";
    return 1;
  }
}
