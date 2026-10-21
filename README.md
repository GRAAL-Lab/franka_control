# Franka Emika Robot control (FER)

ROS2 Jazzy / C++17 control stack for the **original seven-joint Franka Emika Panda**,
with a direct **libfranka** hardware driver and a kinematic simulator.

The package boundaries and command/state architecture follow
`sources/undroid_arm_ctrl`; the ROS2 package, parameter and launch layout follows
`sources/ulisse_core`. Reference projects under `sources/` are not build inputs.

## Packages

| Package | Responsibility |
| --- | --- |
| `fer_msgs` | ROS2 commands, feedback, control and task interfaces; relative topic names |
| `fer_model` | Panda joint limits, forward kinematics and 6×7 Jacobian for simulation |
| `fer_ctrl` | Command validation, state machine, joint and Cartesian velocity control |
| `fer_driver` | Arm FCI worker and independent gripper node, both using libfranka |
| `fer_sim` | Kinematic driver replacement and arm marker visualizer |
| `fer_tcp_bridge` | Optional JSON-lines TCP command service adapter |
| `fer_bringup` | Complete hardware/simulation launch and RViz configuration |

See [the architecture](docs/architecture.md) for the state machine, threading,
interfaces and changes from Undroid.

## Build

Dependencies: ROS2 Jazzy, a C++17 compiler, colcon, Eigen3, the ROS dependencies in
the package manifests, and a libfranka version compatible with the robot firmware.
Follow the [official libfranka installation instructions](https://frankarobotics.github.io/docs/doc/libfranka/README.html)
and [realtime host setup](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html).
CMake locates it with
`find_package(Franka 0.9 REQUIRED)` and links `Franka::Franka`. The hardware build
has been verified against official libfranka **0.9.2**; firmware compatibility must
be checked separately when selecting the installed version.

Run from this repository root, either standalone or inside a ROS2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths fer_* --ignore-src -r -y --skip-keys libfranka
colcon build --base-paths fer_* --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

If libfranka is installed outside the usual prefixes, add
`-DFranka_DIR=/path/to/lib/cmake/Franka` to the CMake arguments.

For development **without libfranka**, build the same packages with the hardware
executables disabled:

```bash
colcon build --base-paths fer_* --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFER_BUILD_HARDWARE=OFF
source install/setup.bash
```

Use `--base-paths fer_*` so colcon does not discover the reference ROS1/ROS2
workspaces in `sources/`. Source `install/setup.bash` in each new terminal.

## Run

Simulation is the default and starts disabled:

```bash
ros2 launch fer_bringup fer.launch.py
# With the arm marker view:
ros2 launch fer_bringup fer.launch.py rviz:=true
```

Hardware:

```bash
ros2 launch fer_bringup fer.launch.py use_sim:=false robot_ip:=172.16.0.2
# For an arm without the Franka hand:
ros2 launch fer_bringup fer.launch.py use_sim:=false robot_ip:=172.16.0.2 with_gripper:=false
```

Unlock the brakes and activate FCI through Franka Desk before enabling. `enable`
starts an FCI joint-velocity session; `disable` ends it after deceleration. These
commands do not control power or mechanical brakes. `recover` explicitly requests
`automaticErrorRecovery()` while the arm is stopped. Network connection failures
require restarting the driver; connection/recovery never resumes an old motion.

Parameters live in `fer_ctrl/conf/controller.yaml`, `fer_driver/conf/driver.yaml`
and `fer_sim/conf/simulator.yaml`. Top-level launch accepts `controller_config`,
`driver_config`, and `simulator_config` paths, plus `namespace` (default `fer`).
The component launch files can also be used independently.

## Commands

The service `/fer/srv/control_command` uses `fer_msgs/srv/ControlCommand`.
Joint indices are **0..6**, joint angles and RPY are **radians**, linear quantities
and gripper width are **metres**, and grasp force is **newtons**.
Responses report acceptance; observe `ctrl/ctrl_data`, `state/state_data`, and
`gripper/state` for progress, errors and completion.

```bash
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: enable}"

# Relative joint movement (joint 1, +0.1 rad).
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: move_joint_i_pos, joint_index: 0, joint_setpoint: [0.1], move_type: relative}"

# Full absolute posture; exactly seven values are required.
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: move_joints_pos, joint_setpoint: [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]}"

# Relative Cartesian displacement expressed in the base frame (BASE=1, EE=0).
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: move_cartesian, move_type: relative, frame_type: 1, target_position: [0.0, 0.0, 0.02]}"

ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: stop}"
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: park}"
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: unpark}"

ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: gripper_move, gripper_setpoint: 0.04, gripper_speed: 0.05}"
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: gripper_grasp, gripper_setpoint: 0.03, grasp_force: 20.0}"
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: gripper_stop}"
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: disable}"
```

Additional commands: `recover`, `gripper_home`, `move_joints_vel` (seven velocities),
and `move_joint_i_vel` (one velocity and index). Refresh velocity commands faster
than `velocity_timeout` (default 0.30 s); otherwise the controller stops in idle.
Position and Cartesian moves have a separate `move_timeout` (30 s).

`park` and `unpark` move to configurable joint postures. Review those postures for
the installed tool and workspace before hardware use. They have no Panda-specific
brake or storage-mode semantics. `stop` is a software motion command; the robot's
physical emergency-stop system remains independent.

## Optional TCP bridge

```bash
ros2 launch fer_bringup fer.launch.py with_tcp:=true tcp_port:=9999
```

The bridge binds to `127.0.0.1` by default (`tcp_host` selects another address).
Send one JSON object per line, using the same fields as `ControlCommand`:

```json
{"command_type":"enable"}
{"command_type":"move_joint_i_pos","joint_index":0,"joint_setpoint":[0.1],"move_type":"relative"}
{"command_type":"stop"}
{"op":"state"}
```

Each line receives a JSON response. `op: state` returns the latest controller and
arm state with feedback age. This is a new seven-joint SI-unit protocol; Undroid's
six-joint binary wire format is not compatible. Line size and request queue are
bounded, and expired queued requests are discarded.

## Validation and scope

```bash
colcon test --base-paths fer_* --packages-select fer_model fer_ctrl fer_driver
colcon test-result --verbose
# Starts simulation and a loopback TCP bridge; no robot connection:
ROS_DOMAIN_ID=171 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/test_simulation.py
```

Tests cover kinematics/Jacobian consistency, commands and state transitions,
timeouts, Cartesian convergence, concurrent data exchange, complete ROS2 launch,
gripper simulation, TCP commands and the driver watchdog/recovery sequence.

The simulator models joint kinematics and bounded acceleration. It does not model
dynamics, contacts, collisions or grasped objects. Hardware feedback uses
`RobotState::O_T_EE` and `Model::zeroJacobian`, so it reflects the end-effector
configuration set on the robot. The simulator defaults to a Panda hand TCP with
0.1034 m offset. The controller uses Eigen damped inverse kinematics and posture
centering; it does not depend on the private RML/FSM/TPIK libraries or implement
Undroid's configurable task-priority/obstacle stack. This model and its limits are
for Panda, not FR3. Physical robot operation has not been validated here.
