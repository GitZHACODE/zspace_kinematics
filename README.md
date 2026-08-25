# zKinematics

A small, standalone C++17 library for serial-robot forward and inverse kinematics.
It is aimed at ABB-style six-axis industrial arms, but operates on any serial KDL chain.
There is no ROS node, ROS runtime, visualization, physics, collision checking, or
RobotStudio dependency.

`joints -> FK -> TCP` and `TCP + current joint seed -> IK -> joints` are the two core
operations. URDF supplies the geometry and joint limits; an optional flange-to-TCP tool
transform is applied in both directions.

## Units

Distances are **metres**. Revolute joint positions are **radians**. Quaternions use the
public ordering **`qw, qx, qy, qz`** and are normalized on input. The core API exposes
no degrees or Euler-angle interfaces.

## Dependencies

The default checkout vendors the source needed for the KDL-only build under
`third_party/`: Orocos KDL, `kdl_parser`, Eigen, URDFDOM, URDFDOM headers,
console_bridge, and TinyXML2. The KDL parser is compiled directly from its upstream
source with a tiny compatibility layer for URDFDOM and diagnostics; it has no ROS
runtime dependency.

Set `ZKINEMATICS_USE_VENDORED_DEPS=OFF` to instead use installed Orocos KDL and
`kdl_parser` packages. On Ubuntu with ROS 2, install the matching distribution
packages, for example:

```bash
sudo apt install liborocos-kdl-dev ros-${ROS_DISTRO}-kdl-parser ros-${ROS_DISTRO}-urdf
```

TRAC-IK is optional. Its upstream source is cloned under `third_party/` and is built
directly, without ROS or catkin, when NLopt and Boost Date-Time are available.
Otherwise KDL's joint-limited Newton-Raphson solver is used.

### Windows: enable TRAC-IK with vcpkg

Install vcpkg and the x64 dependencies:

```powershell
git clone https://github.com/microsoft/vcpkg C:\dev\vcpkg
C:\dev\vcpkg\bootstrap-vcpkg.bat
C:\dev\vcpkg\vcpkg.exe install nlopt:x64-windows boost-date-time:x64-windows
```

The default CMake value for `ZKINEMATICS_VCPKG_INSTALLED_DIR` is
`C:/dev/vcpkg/installed/x64-windows`. Override it when vcpkg is installed elsewhere:

```powershell
cmake -S . -B build -DZKINEMATICS_VCPKG_INSTALLED_DIR=C:/path/to/vcpkg/installed/x64-windows
```

Configuration prints `TRAC-IK found: enabling TRAC-IK backend` when support is active.
The generated `zKinematics.dll` imports `nlopt.dll`; copy both DLLs beside the consuming
executable. The build copies `nlopt.dll` into `build/Release/` automatically.

For a ROS 2 environment, the packaged alternative is:

```bash
sudo apt install ros-${ROS_DISTRO}-trac-ik-lib
```

## Build and test

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Convenience scripts configure with Clang and Ninja, build, then run tests:

```powershell
.\build.bat
```

```bash
./build.sh
```

The same Clang/Ninja configurations are available as CMake presets:

```bash
cmake --preset clang-debug
cmake --build --preset clang-debug
ctest --preset clang-debug
```

Use `clang-release` for an optimised build. Preset build trees are written below
`out/build/`.

By default, `zKinematics` is a shared library. A Visual Studio Release build
places `zKinematics.dll` and its import library (`zKinematics.lib`) in
`build/Release/`, ready for consumers that link and copy from that location. Use
`-DZKINEMATICS_BUILD_SHARED=OFF` to produce a static library instead. The public
`RobotKinematics` class has the required Windows import/export annotation.

## Example

```bash
./build/ikfk_example robot.urdf base_link tool0
```

The example prints chain DOF, ordered joint limits, an FK TCP pose, an IK solution from
a slightly perturbed seed, and final Cartesian errors.

## IK note and limitations

The same TCP pose does not imply a unique joint configuration. Supply the current
physical robot joints as the seed to bias IK toward the desired configuration. Returned
solutions are checked against configured limits and verified with FK.

`RobotKinematics` is intended for single-thread use; separate instances are independent.
This library deliberately has no collision checking, trajectory planning, ABB `confdata`,
RAPID semantics, RobotStudio integration, GUI, or Python bindings. The PIMPL API keeps
KDL types private so pybind11 bindings can be added later.
