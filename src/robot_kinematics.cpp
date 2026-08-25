#include "abb_kinematics/robot_kinematics.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.hpp>

#ifdef ABB_KINEMATICS_HAS_TRAC_IK
#include <trac_ik/trac_ik.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace abb_kinematics {
namespace {

constexpr double kQuaternionEpsilon = 1e-12;
constexpr double kLimitEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

bool isFinite(double value) { return std::isfinite(value); }

void validateFinite(const JointVector &values, const char *description) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!isFinite(values[i])) {
      throw std::invalid_argument(std::string(description) +
                                  " contains NaN or Inf at index " +
                                  std::to_string(i));
    }
  }
}

KDL::Frame poseToKdl(const Pose &pose) {
  if (!isFinite(pose.x) || !isFinite(pose.y) || !isFinite(pose.z) ||
      !isFinite(pose.qw) || !isFinite(pose.qx) || !isFinite(pose.qy) ||
      !isFinite(pose.qz)) {
    throw std::invalid_argument("Pose contains NaN or Inf");
  }
  const double magnitude = std::sqrt(pose.qw * pose.qw + pose.qx * pose.qx +
                                     pose.qy * pose.qy + pose.qz * pose.qz);
  if (magnitude <= kQuaternionEpsilon) {
    throw std::invalid_argument(
        "Pose quaternion has effectively zero magnitude");
  }
  // KDL's Quaternion arguments are x, y, z, w (unlike the public Pose type).
  return KDL::Frame(
      KDL::Rotation::Quaternion(pose.qx / magnitude, pose.qy / magnitude,
                                pose.qz / magnitude, pose.qw / magnitude),
      KDL::Vector(pose.x, pose.y, pose.z));
}

Pose kdlToPose(const KDL::Frame &frame) {
  Pose pose;
  pose.x = frame.p.x();
  pose.y = frame.p.y();
  pose.z = frame.p.z();
  // KDL's GetQuaternion writes x, y, z, w.
  frame.M.GetQuaternion(pose.qx, pose.qy, pose.qz, pose.qw);
  return pose;
}

KDL::JntArray toKdlJoints(const JointVector &joints) {
  KDL::JntArray output(joints.size());
  for (std::size_t i = 0; i < joints.size(); ++i) {
    output(static_cast<unsigned int>(i)) = joints[i];
  }
  return output;
}

JointVector fromKdlJoints(const KDL::JntArray &joints) {
  JointVector output(joints.rows());
  for (unsigned int i = 0; i < joints.rows(); ++i) {
    output[i] = joints(i);
  }
  return output;
}

double orientationError(const KDL::Frame &target, const KDL::Frame &actual) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  (target.M.Inverse() * actual.M).GetQuaternion(x, y, z, w);
  return 2.0 * std::acos(std::clamp(std::abs(w), 0.0, 1.0));
}

double positionError(const KDL::Frame &target, const KDL::Frame &actual) {
  return (target.p - actual.p).Norm();
}

std::string joinFailure(const std::string &prefix, int code) {
  return prefix + " (solver code " + std::to_string(code) + ")";
}

} // namespace

class RobotKinematics::Impl {
public:
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  KDL::JntArray lower_kdl;
  KDL::JntArray upper_kdl;
  JointVector lower_limits;
  JointVector upper_limits;
  std::vector<std::string> joint_names;
  KDL::Frame flange_to_tcp = KDL::Frame::Identity();

  bool withinJointLimits(const JointVector &joints,
                         double epsilon = kLimitEpsilon) const {
    if (joints.size() != lower_limits.size()) {
      return false;
    }
    for (std::size_t i = 0; i < joints.size(); ++i) {
      if (!isFinite(joints[i]) || joints[i] < lower_limits[i] - epsilon ||
          joints[i] > upper_limits[i] + epsilon) {
        return false;
      }
    }
    return true;
  }

  KDL::Frame flangeFk(const JointVector &joints) const {
    KDL::Frame flange;
    const int rc = fk_solver->JntToCart(toKdlJoints(joints), flange);
    if (rc < 0) {
      throw std::runtime_error(
          joinFailure("KDL forward kinematics failed", rc));
    }
    return flange;
  }
};

RobotKinematics::RobotKinematics(const std::string &urdf_file,
                                 const std::string &base_link,
                                 const std::string &flange_link)
    : impl_(std::make_unique<Impl>()) {
  std::ifstream input(urdf_file);
  if (!input) {
    throw std::runtime_error("Unable to read URDF file: " + urdf_file);
  }

  urdf::Model model;
  if (!model.initFile(urdf_file)) {
    throw std::runtime_error("Invalid URDF file: " + urdf_file);
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_file, tree)) {
    throw std::runtime_error("Failed to build KDL tree from URDF: " +
                             urdf_file);
  }
  if (tree.getSegment(base_link) == tree.getSegments().end()) {
    throw std::runtime_error("Unknown base link in URDF: " + base_link);
  }
  if (tree.getSegment(flange_link) == tree.getSegments().end()) {
    throw std::runtime_error("Unknown flange link in URDF: " + flange_link);
  }
  if (!tree.getChain(base_link, flange_link, impl_->chain)) {
    throw std::runtime_error("Failed to construct KDL chain from '" +
                             base_link + "' to '" + flange_link + "'");
  }

  for (unsigned int segment_index = 0;
       segment_index < impl_->chain.getNrOfSegments(); ++segment_index) {
    const KDL::Joint &kdl_joint =
        impl_->chain.getSegment(segment_index).getJoint();
    if (kdl_joint.getType() == KDL::Joint::None) {
      continue;
    }
    const std::string joint_name = kdl_joint.getName();
    const auto urdf_joint = model.getJoint(joint_name);
    if (!urdf_joint) {
      throw std::runtime_error("Malformed URDF: KDL joint '" + joint_name +
                               "' is absent from the URDF model");
    }

    double lower = 0.0;
    double upper = 0.0;
    const bool has_finite_limits = urdf_joint->limits &&
                                   isFinite(urdf_joint->limits->lower) &&
                                   isFinite(urdf_joint->limits->upper);
    if (urdf_joint->type == urdf::Joint::CONTINUOUS) {
      lower = has_finite_limits ? urdf_joint->limits->lower : -kPi;
      upper = has_finite_limits ? urdf_joint->limits->upper : kPi;
    } else {
      if (!has_finite_limits) {
        throw std::runtime_error("Missing finite limits for movable joint '" +
                                 joint_name + "'");
      }
      lower = urdf_joint->limits->lower;
      upper = urdf_joint->limits->upper;
    }
    if (lower > upper) {
      throw std::runtime_error("Malformed limits for joint '" + joint_name +
                               "'");
    }
    impl_->joint_names.push_back(joint_name);
    impl_->lower_limits.push_back(lower);
    impl_->upper_limits.push_back(upper);
  }

  if (impl_->joint_names.empty()) {
    throw std::runtime_error("KDL chain contains no movable joints");
  }
  if (impl_->joint_names.size() != impl_->chain.getNrOfJoints()) {
    throw std::runtime_error("Malformed model: KDL chain joint count does not "
                             "match its movable joints");
  }

  impl_->lower_kdl = toKdlJoints(impl_->lower_limits);
  impl_->upper_kdl = toKdlJoints(impl_->upper_limits);
  impl_->fk_solver =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(impl_->chain);
}

RobotKinematics::~RobotKinematics() = default;
RobotKinematics::RobotKinematics(RobotKinematics &&) noexcept = default;
RobotKinematics &
RobotKinematics::operator=(RobotKinematics &&) noexcept = default;

std::size_t RobotKinematics::dof() const noexcept {
  return impl_->joint_names.size();
}
const JointVector &RobotKinematics::lowerLimits() const noexcept {
  return impl_->lower_limits;
}
const JointVector &RobotKinematics::upperLimits() const noexcept {
  return impl_->upper_limits;
}
const std::vector<std::string> &RobotKinematics::jointNames() const noexcept {
  return impl_->joint_names;
}

void RobotKinematics::setTool(const Pose &flange_to_tcp) {
  impl_->flange_to_tcp = poseToKdl(flange_to_tcp);
}

Pose RobotKinematics::tool() const { return kdlToPose(impl_->flange_to_tcp); }

Pose RobotKinematics::fk(const JointVector &joints) const {
  if (joints.size() != dof()) {
    throw std::invalid_argument("FK expected " + std::to_string(dof()) +
                                " joints, received " +
                                std::to_string(joints.size()));
  }
  validateFinite(joints, "FK joint vector");
  return kdlToPose(impl_->flangeFk(joints) * impl_->flange_to_tcp);
}

IKResult RobotKinematics::ik(const Pose &target_tcp, const JointVector &seed,
                             const IKOptions &options) const {
  if (seed.size() != dof()) {
    throw std::invalid_argument("IK seed expected " + std::to_string(dof()) +
                                " joints, received " +
                                std::to_string(seed.size()));
  }
  validateFinite(seed, "IK seed");
  if (!impl_->withinJointLimits(seed)) {
    throw std::invalid_argument("IK seed is outside configured joint limits");
  }
  if (!isFinite(options.timeout_seconds) || options.timeout_seconds <= 0.0 ||
      !isFinite(options.tolerance) || options.tolerance <= 0.0 ||
      options.max_iterations == 0) {
    throw std::invalid_argument("IK options require positive finite "
                                "timeout/tolerance and max_iterations");
  }

  const KDL::Frame target_flange =
      poseToKdl(target_tcp) * impl_->flange_to_tcp.Inverse();
  KDL::JntArray output(static_cast<unsigned int>(dof()));
  int rc = -1;

  if (options.backend == IKBackend::KDL) {
    KDL::ChainIkSolverVel_pinv velocity_solver(impl_->chain, options.tolerance);
    KDL::ChainIkSolverPos_NR_JL position_solver(
        impl_->chain, impl_->lower_kdl, impl_->upper_kdl, *impl_->fk_solver,
        velocity_solver, options.max_iterations, options.tolerance);
    rc = position_solver.CartToJnt(toKdlJoints(seed), target_flange, output);
  } else {
#ifdef ABB_KINEMATICS_HAS_TRAC_IK
    TRAC_IK::TRAC_IK solver(impl_->chain, impl_->lower_kdl, impl_->upper_kdl,
                            options.timeout_seconds, options.tolerance,
                            TRAC_IK::Distance);
    rc = solver.CartToJnt(toKdlJoints(seed), target_flange, output);
#else
    return {false,
            {},
            -1,
            "TRAC-IK was requested but this library was built without TRAC-IK"};
#endif
  }

  if (rc < 0) {
    return {
        false, {}, rc, joinFailure("IK solver did not find a solution", rc)};
  }
  JointVector solution = fromKdlJoints(output);
  for (std::size_t i = 0; i < solution.size(); ++i) {
    if (!isFinite(solution[i])) {
      return {false,
              {},
              rc,
              "IK solver returned NaN or Inf for joint " + std::to_string(i)};
    }
    if (solution[i] < impl_->lower_limits[i] - kLimitEpsilon ||
        solution[i] > impl_->upper_limits[i] + kLimitEpsilon) {
      return {false,
              {},
              rc,
              "IK solver returned joint " + std::to_string(i) +
                  " outside configured limits"};
    }
  }

  const KDL::Frame actual_flange = impl_->flangeFk(solution);
  const double pos_error = positionError(target_flange, actual_flange);
  const double rot_error = orientationError(target_flange, actual_flange);
  if (pos_error > options.tolerance || rot_error > options.tolerance) {
    std::ostringstream message;
    message << "IK solver result failed validation (position error "
            << pos_error << " m, orientation error " << rot_error << " rad)";
    return {false, {}, rc, message.str()};
  }
  return {true, std::move(solution), rc, "IK solution found"};
}

} // namespace abb_kinematics
