#pragma once

#include <string>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.hpp>

namespace urdf {

// ROS's urdf package adds this convenience wrapper over URDFDOM.  kdl_parser
// uses only these parsing and lookup operations, so reproducing them here
// avoids a ROS runtime dependency while preserving its upstream API.
class Model {
public:
  bool initFile(const std::string &path) {
    model_ = parseURDFFile(path);
    return static_cast<bool>(model_);
  }

  bool initString(const char *xml) {
    model_ = parseURDF(xml == nullptr ? std::string{} : std::string(xml));
    return static_cast<bool>(model_);
  }

  JointConstSharedPtr getJoint(const std::string &name) const {
    return model_ ? model_->getJoint(name) : JointConstSharedPtr{};
  }

  operator const ModelInterface &() const { return *model_; }

private:
  ModelInterfaceSharedPtr model_;
};

} // namespace urdf
