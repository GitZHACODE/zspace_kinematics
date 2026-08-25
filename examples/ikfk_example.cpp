#include "zKinematics/robot_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

double positionError(const zKinematics::Pose &a, const zKinematics::Pose &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientationError(const zKinematics::Pose &a,
                        const zKinematics::Pose &b) {
  const double dot =
      std::abs(a.qw * b.qw + a.qx * b.qx + a.qy * b.qy + a.qz * b.qz);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

} // namespace

int main(int argc, char *argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " robot.urdf base_link flange_link\n";
    return 2;
  }

  try {
    zKinematics::RobotKinematics robot(argv[1], argv[2], argv[3]);
    std::cout << "DOF: " << robot.dof() << "\n";
    zKinematics::JointVector joints(robot.dof());
    for (std::size_t i = 0; i < robot.dof(); ++i) {
      const double lower = robot.lowerLimits()[i];
      const double upper = robot.upperLimits()[i];
      joints[i] = 0.5 * (lower + upper);
      std::cout << "  " << robot.jointNames()[i] << ": [" << lower << ", "
                << upper << "]\n";
    }

    const auto target = robot.fk(joints);
    std::cout << std::setprecision(12) << "FK TCP: " << target.x << ' '
              << target.y << ' ' << target.z << ' ' << target.qw << ' '
              << target.qx << ' ' << target.qy << ' ' << target.qz << "\n";

    auto seed = joints;
    for (std::size_t i = 0; i < seed.size(); ++i) {
      seed[i] = std::clamp(seed[i] + 1e-3, robot.lowerLimits()[i],
                           robot.upperLimits()[i]);
    }
    const auto result = robot.ik(target, seed);
    if (!result.success) {
      std::cerr << "IK failed: " << result.message << "\n";
      return 1;
    }
    std::cout << "IK joints:";
    for (double joint : result.joints)
      std::cout << ' ' << joint;
    std::cout << "\n";

    const auto actual = robot.fk(result.joints);
    std::cout << "IK validation: position error "
              << positionError(target, actual) << " m, orientation error "
              << orientationError(target, actual) << " rad\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << "\n";
    return 1;
  }
}
