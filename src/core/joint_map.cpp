#include "go2w_standup/core/joint_map.hpp"

#include <stdexcept>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

namespace go2w_standup {

namespace {

Leg ParseLeg(const std::string& s) {
  if (s == "FR") return Leg::kFR;
  if (s == "FL") return Leg::kFL;
  if (s == "RR") return Leg::kRR;
  if (s == "RL") return Leg::kRL;
  throw std::runtime_error("JointMap: unknown leg '" + s + "'");
}

JointType ParseType(const std::string& s) {
  if (s == "hip") return JointType::kHip;
  if (s == "thigh") return JointType::kThigh;
  if (s == "calf") return JointType::kCalf;
  if (s == "wheel") return JointType::kWheel;
  throw std::runtime_error("JointMap: unknown joint type '" + s + "'");
}

int LegOrdinal(Leg leg) { return static_cast<int>(leg); }  // FR=0..RL=3

}  // namespace

JointMap JointMap::LoadFromYaml(const std::string& robot_yaml_path) {
  const YAML::Node root = YAML::LoadFile(robot_yaml_path);
  const YAML::Node map_node = root["joint_map"];
  if (!map_node) {
    throw std::runtime_error("JointMap: 'joint_map' missing in " +
                             robot_yaml_path);
  }

  JointMap m;
  std::unordered_set<std::string> names;
  int count = 0;
  for (const auto& kv : map_node) {
    const int idx = kv.first.as<int>();
    if (idx < 0 || idx >= kNumJoints) {
      throw std::runtime_error("JointMap: sdk_index out of range: " +
                               std::to_string(idx));
    }
    const YAML::Node& e = kv.second;
    JointInfo info;
    info.sdk_index = idx;
    info.canonical_name = e["name"].as<std::string>();
    // V1: runtime canonical names are identical to MJCF joint names
    // (go2w.xml already uses *_wheel_joint, spec §11).
    info.mjcf_name = e["mjcf_name"] ? e["mjcf_name"].as<std::string>()
                                    : info.canonical_name;
    info.axis = e["axis"] ? e["axis"].as<std::string>() : "";
    info.leg = ParseLeg(e["leg"].as<std::string>());
    info.type = ParseType(e["type"].as<std::string>());

    if (!names.insert(info.canonical_name).second) {
      throw std::runtime_error("JointMap: duplicate name '" +
                               info.canonical_name + "'");
    }
    if (!m.joints_[idx].canonical_name.empty()) {
      throw std::runtime_error("JointMap: duplicate sdk_index " +
                               std::to_string(idx));
    }
    m.joints_[idx] = info;
    ++count;
  }
  if (count != kNumJoints) {
    throw std::runtime_error("JointMap: expected 16 entries, got " +
                             std::to_string(count));
  }

  const YAML::Node wfs = root["wheel_forward_sign"];
  if (!wfs) {
    throw std::runtime_error("JointMap: 'wheel_forward_sign' missing in " +
                             robot_yaml_path);
  }
  for (const char* l : {"FR", "FL", "RR", "RL"}) {
    const double s = wfs[l].as<double>();
    if (s != 1.0 && s != -1.0) {
      throw std::runtime_error(std::string("JointMap: wheel_forward_sign.") +
                               l + " must be +/-1");
    }
    m.wheel_forward_sign_[LegOrdinal(ParseLeg(l))] = s;
  }
  return m;
}

const JointInfo& JointMap::at(int sdk_index) const {
  if (sdk_index < 0 || sdk_index >= kNumJoints) {
    throw std::out_of_range("JointMap::at " + std::to_string(sdk_index));
  }
  return joints_[sdk_index];
}

int JointMap::sdkIndex(const std::string& canonical_name) const {
  for (const auto& j : joints_) {
    if (j.canonical_name == canonical_name) return j.sdk_index;
  }
  throw std::runtime_error("JointMap: unknown joint '" + canonical_name + "'");
}

double JointMap::wheelForwardSign(Leg leg) const {
  return wheel_forward_sign_[LegOrdinal(leg)];
}

const char* JointMap::LegName(Leg leg) {
  switch (leg) {
    case Leg::kFR: return "FR";
    case Leg::kFL: return "FL";
    case Leg::kRR: return "RR";
    case Leg::kRL: return "RL";
  }
  return "?";
}

const char* JointMap::TypeName(JointType type) {
  switch (type) {
    case JointType::kHip: return "hip";
    case JointType::kThigh: return "thigh";
    case JointType::kCalf: return "calf";
    case JointType::kWheel: return "wheel";
  }
  return "?";
}

}  // namespace go2w_standup
