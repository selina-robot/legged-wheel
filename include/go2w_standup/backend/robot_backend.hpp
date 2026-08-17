// RobotBackend abstraction (spec §10). Upper layers never branch on
// simulation vs real hardware; only the backend config differs.
#pragma once

#include "go2w_standup/core/types.hpp"

namespace go2w_standup {

class RobotBackend {
 public:
  virtual ~RobotBackend() = default;

  virtual bool initialize() = 0;
  virtual bool read(RobotState& state) = 0;
  virtual bool write(const MotorCommand& cmd) = 0;
  virtual void emergencyDamping() = 0;
};

}  // namespace go2w_standup
