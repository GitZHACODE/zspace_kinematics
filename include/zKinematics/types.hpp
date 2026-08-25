#pragma once

#include <string>
#include <vector>

namespace zKinematics {

using JointVector = std::vector<double>;

struct Pose {
  // Metres.
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  // Unit quaternion, scalar first.
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
};

enum class IKBackend {
  KDL,
  TRAC_IK,
};

struct IKOptions {
  // Maximum solver time where supported.
  double timeout_seconds = 0.005;
  // Cartesian convergence tolerance in metres/radians.
  double tolerance = 1e-5;
  // Used by the KDL iterative solver.
  unsigned int max_iterations = 200;
#ifdef ZKINEMATICS_HAS_TRAC_IK
  IKBackend backend = IKBackend::TRAC_IK;
#else
  IKBackend backend = IKBackend::KDL;
#endif
};

struct IKResult {
  bool success = false;
  JointVector joints;
  // Raw return code from the underlying solver when applicable.
  int solver_code = -1;
  std::string message;
};

}  // namespace zKinematics
