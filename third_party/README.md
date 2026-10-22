# Vendored dependencies

The default CMake configuration builds Orocos KDL and `kdl_parser` from the cloned
upstream sources in this directory. Their standalone build dependencies are vendored
alongside them: Eigen, URDFDOM, URDFDOM headers, console_bridge, and TinyXML2.

`standalone_compat/` contains narrowly scoped compatibility headers that let the
upstream ROS-packaged KDL parser use URDFDOM directly and emit diagnostics without a
ROS runtime.

Initialize all pinned upstream checkouts with `git submodule update --init --recursive`.

TRAC-IK is built directly from its upstream sources when Boost Date-Time and NLopt
are available, using the compatibility headers without ROS or catkin. Otherwise,
the KDL joint-limited backend remains available.
