#include "abb_kinematics/robot_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using abb_kinematics::JointVector;
using abb_kinematics::Pose;
using abb_kinematics::RobotKinematics;

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

double positionError(const Pose &a, const Pose &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientationError(const Pose &a, const Pose &b) {
  const double dot =
      std::abs(a.qw * b.qw + a.qx * b.qx + a.qy * b.qy + a.qz * b.qz);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

void requireFinite(const Pose &pose) {
  require(std::isfinite(pose.x) && std::isfinite(pose.y) &&
              std::isfinite(pose.z) && std::isfinite(pose.qw) &&
              std::isfinite(pose.qx) && std::isfinite(pose.qy) &&
              std::isfinite(pose.qz),
          "Pose contains a non-finite value");
}

void requireThrows(const std::function<void()> &operation) {
  try {
    operation();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error("Expected an exception");
}

RobotKinematics loadRobot() {
  return RobotKinematics(ABB_KINEMATICS_TEST_URDF, "base_link", "tool0");
}

abb_kinematics::IKOptions kdlOptions() {
  abb_kinematics::IKOptions options;
  options.backend = abb_kinematics::IKBackend::KDL;
  options.max_iterations = 1000;
  options.tolerance = 1e-6;
  return options;
}

void testUrdfLoading() {
  auto robot = loadRobot();
  require(robot.dof() == 6, "Expected six DOF");
  const std::vector<std::string> expected{"joint_1", "joint_2", "joint_3",
                                          "joint_4", "joint_5", "joint_6"};
  require(robot.jointNames() == expected, "Joint names are not in chain order");
  require(robot.lowerLimits().size() == 6 && robot.upperLimits().size() == 6,
          "Joint limits have wrong size");
  require(std::abs(robot.lowerLimits()[0] + 2.8) < 1e-12 &&
              std::abs(robot.upperLimits()[5] - 2.8) < 1e-12,
          "Joint limits were not loaded");
}

void testFk() {
  auto robot = loadRobot();
  requireFinite(robot.fk({0.2, -0.3, 0.4, -0.2, 0.3, -0.1}));
}

void testRoundTripsAndLimits() {
  auto robot = loadRobot();
  const std::vector<JointVector> configurations{
      {0.1, -0.2, 0.3, 0.2, -0.3, 0.1},
      {-0.4, 0.25, -0.35, 0.3, 0.2, -0.15},
      {0.35, 0.15, 0.2, -0.25, 0.25, 0.3}};
  for (const auto &original : configurations) {
    const Pose target = robot.fk(original);
    JointVector seed = original;
    for (std::size_t i = 0; i < seed.size(); ++i)
      seed[i] += (i % 2 == 0 ? 1e-3 : -1e-3);
    const auto result = robot.ik(target, seed, kdlOptions());
    require(result.success, "Round-trip IK failed: " + result.message);
    require(positionError(target, robot.fk(result.joints)) < 1e-4,
            "Round-trip position error too large");
    require(orientationError(target, robot.fk(result.joints)) < 1e-4,
            "Round-trip orientation error too large");
    for (std::size_t i = 0; i < result.joints.size(); ++i) {
      require(result.joints[i] >= robot.lowerLimits()[i] - 1e-9 &&
                  result.joints[i] <= robot.upperLimits()[i] + 1e-9,
              "IK result violates joint limits");
    }
  }
}

void testToolTransform() {
  auto robot = loadRobot();
  const JointVector joints{0.2, -0.2, 0.3, 0.1, -0.2, 0.15};
  const Pose flange = robot.fk(joints);
  robot.setTool({0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0});
  const Pose tcp = robot.fk(joints);
  require(std::abs(positionError(flange, tcp) - 0.2) < 1e-9,
          "Tool did not shift TCP by 0.2 m");
  const auto result = robot.ik(tcp, joints, kdlOptions());
  require(result.success, "IK with tool failed: " + result.message);
  require(positionError(tcp, robot.fk(result.joints)) < 1e-4,
          "IK did not compensate tool transform");
}

void testInvalidInput() {
  auto robot = loadRobot();
  requireThrows([&] { robot.fk({0.0}); });
  requireThrows([&] { robot.ik(robot.fk({0, 0, 0, 0, 0, 0}), {0.0}); });
  requireThrows([&] { robot.setTool({0, 0, 0, 0, 0, 0, 0}); });
  requireThrows([&] { robot.ik({0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}); });
  requireThrows([] { RobotKinematics("missing.urdf", "base_link", "tool0"); });
}

void testUnreachableTarget() {
  auto robot = loadRobot();
  const auto result = robot.ik({100.0, 100.0, 100.0, 1.0, 0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, kdlOptions());
  require(!result.success, "Unreachable target unexpectedly succeeded");
}

} // namespace

int main() {
  const std::vector<std::pair<std::string, std::function<void()>>> tests{
      {"URDF loading", testUrdfLoading},
      {"FK", testFk},
      {"FK -> IK -> FK", testRoundTripsAndLimits},
      {"tool transform", testToolTransform},
      {"invalid input", testInvalidInput},
      {"unreachable target", testUnreachableTarget}};
  try {
    for (const auto &test : tests) {
      test.second();
      std::cout << "PASS: " << test.first << '\n';
    }
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
