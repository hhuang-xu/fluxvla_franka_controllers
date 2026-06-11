# FluxVLA Franka Controllers

`fluxvla_franka_controllers` is a ROS Noetic controller package for Franka
Emika robots. It contains Cartesian impedance control, Ruckig-smoothed joint
position control, and a Ruckig-based joint impedance controller.

This package is forked from the upstream SERL Franka controller package. The
original MIT license and attribution are preserved in `LICENSE` and `NOTICE`.

## Controllers

- `cartesian_impedance_controller`: Cartesian impedance controller with dynamic
  reconfigure compliance parameters.
- `joint_position_controller`: simple reset / point-to-point joint position
  controller.
- `ruckig_joint_position_controller`: PositionJointInterface controller that
  accepts `sensor_msgs/JointState` targets on `target_joint_state` and smooths
  them with Ruckig.
- `ruckig_joint_impedance_controller`: EffortJointInterface controller that
  accepts joint targets, smooths them with Ruckig, and outputs follower-style
  joint PD torques.

## Prerequisites

Use Ubuntu 20.04 with ROS Noetic and a catkin workspace.

```bash
sudo apt update
sudo apt install \
  build-essential cmake git python3-catkin-tools python3-rosdep \
  ros-noetic-controller-interface \
  ros-noetic-controller-manager \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-eigen-conversions \
  ros-noetic-franka-ros \
  ros-noetic-hardware-interface \
  ros-noetic-pluginlib \
  ros-noetic-realtime-tools \
  ros-noetic-ros-control \
  ros-noetic-rqt-reconfigure \
  ros-noetic-tf \
  ros-noetic-tf-conversions
```

Install `libfranka` and `franka_ros` according to the Franka FCI documentation.
On machines using the ROS packages, the command above installs the Noetic
`franka_ros` stack and its packaged `libfranka` dependency.

## Install Ruckig

Prefer the ROS package if it is available in your apt sources:

```bash
sudo apt install ros-noetic-ruckig
```

If the package is not available, install Ruckig from source so CMake can find
the imported target `ruckig::ruckig`:

```bash
cd /tmp
git clone https://github.com/pantor/ruckig.git
cd ruckig
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PYTHON_MODULE=OFF
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig
```

If Ruckig is installed to a custom prefix, source or export that prefix before
building this package:

```bash
export CMAKE_PREFIX_PATH=/path/to/ruckig/install:$CMAKE_PREFIX_PATH
```

## Build From Source

Place this repository in a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <your-fluxvla-franka-controllers-repo> fluxvla_franka_controllers
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make --pkg fluxvla_franka_controllers
source devel/setup.bash
```

Verify that ROS can find the package:

```bash
rospack find fluxvla_franka_controllers
roscd fluxvla_franka_controllers
```

## Launch Controllers

Replace `<RobotIP>` with the Franka robot IP address. Set `load_gripper` to
`true` only when a gripper should be loaded.

Launch files for replay:

```bash
roslaunch fluxvla_franka_controllers single_joint.launch \
  robot_ip:=<RobotIP> load_gripper:=false
roslaunch fluxvla_franka_controllers single_eepose.launch \
  robot_ip:=<RobotIP> load_gripper:=false
roslaunch fluxvla_franka_controllers dual_joint.launch \
  left_robot_ip:=<LeftIP> right_robot_ip:=<RightIP> load_gripper:=false
roslaunch fluxvla_franka_controllers dual_eepose.launch \
  left_robot_ip:=<LeftIP> right_robot_ip:=<RightIP> load_gripper:=false
```

The `joint` examples launch `fluxvla_franka_controllers/RuckigJointImpedanceController`.
The `eepose` examples are provided here as convenience entry points and launch
SERL's `serl_franka_controllers/CartesianImpedanceController`.

After launching a replay controller, publish targets to:

```text
/<arm_namespace>/ruckig_joint_impedance_controller/target_joint_state
/<arm_namespace>/cartesian_impedance_controller/equilibrium_pose
```

For a single-arm launch without an outer namespace, omit the arm namespace.

## Basic Verification

Check that the controller is loaded:

```bash
rosservice call /controller_manager/list_controllers
```

Replay a prepared dual-arm CSV through the joint impedance controller at 30 Hz:

```bash
rosrun fluxvla_franka_controllers replay_joint_impedance_csv.py \
  --csv /path/to/replay.csv --control_mode joint --execute
```

Replay end-effector poses through the SERL Cartesian impedance controller:

```bash
rosrun fluxvla_franka_controllers replay_joint_impedance_csv.py \
  --csv /path/to/replay.csv --control_mode eepose --execute
```

Joint mode expects `left_q1` ... `left_q7` and `right_q1` ... `right_q7`.
End-effector mode expects `left_x,left_y,left_z,left_qx,left_qy,left_qz,left_qw`
and the matching `right_*` columns. Optional `left_gripper` and `right_gripper`
columns are published to the Franka gripper move action goal topics.
Dataset-specific conversion, such as parquet to CSV, should live outside this
controller package.

## Realtime Note

`franka_ros` expects a realtime-capable system for robot control. If you are
developing on a non-realtime machine, follow the Franka documentation for safe
setup. For local testing without hardware, do not connect to the robot.

Some development machines ignore the realtime check in
`franka_control/config/franka_control_node.yaml`:

```yaml
realtime_config: ignore
```

Only use that workaround if it matches your lab's safety policy.
