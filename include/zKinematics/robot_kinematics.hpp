#pragma once

#include "zKinematics/types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace zKinematics {

#if defined(_WIN32)
#if defined(ZKINEMATICS_STATIC)
#define ZKINEMATICS_API
#elif defined(ZKINEMATICS_BUILDING_DLL)
#define ZKINEMATICS_API __declspec(dllexport)
#elif defined(ZKINEMATICS_SHARED)
#define ZKINEMATICS_API __declspec(dllimport)
#else
#define ZKINEMATICS_API
#endif
#else
#define ZKINEMATICS_API __attribute__((visibility("default")))
#endif

class ZKINEMATICS_API RobotKinematics {
 public:
  RobotKinematics(const std::string& urdf_file, const std::string& base_link,
                  const std::string& flange_link);
  ~RobotKinematics();

  RobotKinematics(const RobotKinematics&) = delete;
  RobotKinematics& operator=(const RobotKinematics&) = delete;
  RobotKinematics(RobotKinematics&&) noexcept;
  RobotKinematics& operator=(RobotKinematics&&) noexcept;

  std::size_t dof() const noexcept;
  const JointVector& lowerLimits() const noexcept;
  const JointVector& upperLimits() const noexcept;
  const std::vector<std::string>& jointNames() const noexcept;

  // Flange -> TCP transform.
  void setTool(const Pose& flange_to_tcp);
  Pose tool() const;

  // Joint angles (radians) -> TCP pose (metres and qw,qx,qy,qz quaternion).
  Pose fk(const JointVector& joints) const;

  // TCP target + seed -> joint angles (radians).
  IKResult ik(const Pose& target_tcp, const JointVector& seed,
              const IKOptions& options = {}) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace zKinematics
