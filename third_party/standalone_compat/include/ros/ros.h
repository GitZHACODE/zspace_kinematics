#pragma once

// The chain-based TRAC-IK constructor used by zKinematics does not require a
// ROS runtime.  Its upstream implementation includes ros/ros.h solely for
// diagnostics, so provide no-op standalone replacements for those macros.
#define ROS_DEBUG_NAMED(...) ((void)0)
#define ROS_DEBUG_STREAM_NAMED(...) ((void)0)
#define ROS_ERROR(...) ((void)0)
#define ROS_ERROR_THROTTLE(...) ((void)0)
#define ROS_FATAL(...) ((void)0)
#define ROS_FATAL_NAMED(...) ((void)0)
#define ROS_FATAL_STREAM(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
