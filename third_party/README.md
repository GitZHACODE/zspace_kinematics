# Vendored dependencies

The default CMake configuration builds Orocos KDL and `kdl_parser` from the cloned
upstream sources in this directory. Their standalone build dependencies are vendored
alongside them: Eigen, URDFDOM, URDFDOM headers, console_bridge, and TinyXML2.

`standalone_compat/` contains narrowly scoped compatibility headers that let the
upstream ROS-packaged KDL parser use URDFDOM directly and emit diagnostics without a
ROS runtime.

TRAC-IK is cloned for optional use, but its upstream checkout currently requires its
own Boost, NLopt, and ROS/catkin packaging dependencies. It is therefore not enabled
by default; the KDL joint-limited backend always remains available.
