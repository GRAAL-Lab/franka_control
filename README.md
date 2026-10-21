# Franka Emika Robot control (FER)

ROS2 Jazzy / C++17 control stack for the seven-joint Franka Emika Panda, with a
task-priority kinematic controller, a joint dynamic controller, a direct libfranka
driver, and a simulator.

## Controller configuration

The two controller configurations are:

- [kcl_fer.conf](fer_ctrl/conf/kcl_fer.conf): tasks, activation parameters, gains,
  saturations, priority levels, actions, state/action mappings, and park/unpark postures.
  It uses the same libconfig structure as Undroid's `kcl_undroid.conf`.
- [dcl_fer.conf](fer_ctrl/conf/dcl_fer.conf): computed-torque or joint-impedance
  control, joint gains, reference velocity/acceleration/jerk limits, torque limits,
  and torque-rate limits.

The control chain is:

```text
ControlCommand -> KinematicController / FSM / TPIK
               -> joint velocity reference
               -> DynamicControlNode / reference generator
               -> joint position, velocity and acceleration references
               -> DynamicController::ComputeTorques() in the driver
               -> franka::Torques
```

`dynamic_control_node` runs the ROS reference generator at 500 Hz.
The hardware driver evaluates the dynamic control law at libfranka's 1 kHz rate
using the current measured joints, mass matrix, and Coriolis vector.
The simulator uses the same dynamic-controller library.

## Packages and source layout

| Package | Responsibility |
| --- | --- |
| `fer_msgs` | ROS2 commands, feedback, dynamic references, task status and TPIK action messages |
| `fer_model` | Panda kinematics and `FrankaArmModel`, the RML model used by IKCL |
| `fer_ctrl` | Kinematic controller, command/state classes, configuration loaders and dynamic controller |
| `fer_driver` | libfranka arm torque driver and independent gripper node |
| `fer_sim` | Joint-dynamics simulator and arm marker visualizer |
| `fer_tcp_bridge` | Optional JSON-lines TCP adapter for the command service |
| `fer_bringup` | Complete hardware/simulation launch and RViz configuration |

Headers and implementations are separated into `include/<package>/` and
`src/<package>/`. Commands and states follow Undroid's `GenericCommand` and
`GenericState` organization. Functions use names such as `Initialization`,
`SetUpFSM`, `StateDataCB`, `CommandsHandler`, `Run`, and `PublishControl`.
Executable source files contain the ROS initialization and spin loop.
[.clang-format](.clang-format) records the common C++ style.

See [the architecture](docs/architecture.md) for the state machine, controller
equations, ROS interfaces, and realtime boundary.

## Dependencies and build

Install ROS2 Jazzy, colcon, Eigen3 and `libconfig++-dev`. Build and install the
same GRAAL libraries used by the reference controller: `rml`, `fsm`,
`ctrl_toolbox`, `tpik`, and `ikcl`. Their CMake packages must be discoverable
through the normal prefixes or `CMAKE_PREFIX_PATH`.

For hardware, install a libfranka version compatible with the robot firmware.
See the [official libfranka installation instructions](https://frankarobotics.github.io/docs/doc/libfranka/README.html)
and [realtime host setup](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html).
The driver links `Franka::Franka`; its hardware build is checked against
libfranka 0.9.2.

Run from this repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths fer_* --ignore-src -r -y \
  --skip-keys "libfranka rml fsm ctrl_toolbox tpik ikcl"
colcon build --base-paths fer_* --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

For a custom libfranka installation, append
`-DFranka_DIR=/path/to/lib/cmake/Franka` to the CMake arguments.

To build for simulation without libfranka:

```bash
colcon build --base-paths fer_* --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFER_BUILD_HARDWARE=OFF
source install/setup.bash
```

The GRAAL controller libraries are required in both builds. Source the installed
workspace in each terminal. The explicit base paths keep the reference workspaces
under `sources/` out of the build.

## Run

Simulation starts both controllers and the simulated driver, initially disabled:

```bash
ros2 launch fer_bringup fer.launch.py
# Add the arm marker view:
ros2 launch fer_bringup fer.launch.py rviz:=true
```

Select controller configurations explicitly:

```bash
ros2 launch fer_bringup fer.launch.py \
  kcl_config:=/absolute/path/kcl_fer.conf \
  dcl_config:=/absolute/path/dcl_fer.conf
```

The same `dcl_config` is passed to the dynamic node and selected backend.
Configuration files are loaded at startup; restart the stack after changing them.

Hardware:

```bash
ros2 launch fer_bringup fer.launch.py use_sim:=false robot_ip:=172.16.0.2
# Without the Franka hand:
ros2 launch fer_bringup fer.launch.py use_sim:=false robot_ip:=172.16.0.2 with_gripper:=false
```

Activate FCI and unlock the brakes in Franka Desk before enabling.
`enable` starts torque control; `disable` decelerates the joint reference and
ends the session. These commands do not operate power or brakes.
`recover` requests `automaticErrorRecovery()` after control has stopped.
Connection failures require restarting the driver. Recovery and reconnection
never resume an old motion.

ROS parameters for frames and driver/simulator settings remain in
`fer_ctrl/conf/controller.yaml`, `fer_driver/conf/driver.yaml`, and
`fer_sim/conf/simulator.yaml`. Top-level launch also accepts `controller_config`,
`driver_config`, `simulator_config`, and `namespace` (default `fer`).

## Commands and monitoring

Use `/fer/srv/control_command` with `fer_msgs/srv/ControlCommand`.
Joint indices are 0..6; joint angles and RPY are radians, distances and gripper
width are metres, and grasp force is newtons. Responses acknowledge acceptance;
observe state and control topics for completion.

```bash
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: enable}"

ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: move_joint_i_pos, joint_index: 0, joint_setpoint: [0.1], move_type: relative}"

ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: move_cartesian, move_type: relative, frame_type: 1, target_position: [0.0, 0.0, 0.02]}"

ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: stop}"
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: park}"
ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: unpark}"

ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand \
  "{command_type: gripper_move, gripper_setpoint: 0.04, gripper_speed: 0.05}"

ros2 service call /fer/srv/control_command fer_msgs/srv/ControlCommand "{command_type: disable}"

# Configured action, ordered priority levels and task names:
ros2 topic echo /fer/ctrl/tpik_action
# Per-task activation functions and reference rates:
ros2 topic echo /fer/tasks/JointsLimit
# Dynamic references and actual commanded torques:
ros2 topic echo /fer/ctrl/dynamic_control
ros2 topic echo /fer/state/state_data
```

Other commands: `move_joints_pos`, `move_joints_vel`, `move_joint_i_vel`,
`recover`, `gripper_grasp`, `gripper_home`, and `gripper_stop`.
Full joint requests require seven values. Refresh velocity commands before
`velocityCommandTimeout` expires; otherwise the controller returns to idle.
Position and Cartesian moves use `moveTimeout`.

Cartesian frames are EE=0 and BASE=1. Absolute poses use BASE. Relative BASE
requests add a base-frame translation and premultiply the rotation; relative EE
requests postmultiply the tool transform.

Park/unpark move to configured joint postures. Check these postures and the
configured obstacle planes against the installed workspace and tool.
The software `stop` command is separate from the physical emergency-stop system.

## Optional TCP bridge

```bash
ros2 launch fer_bringup fer.launch.py with_tcp:=true tcp_port:=9999
```

The default address is `127.0.0.1`; `tcp_host` selects another address.
Send one JSON request per line, using the `ControlCommand` fields:

```json
{"command_type":"enable"}
{"command_type":"move_joint_i_pos","joint_index":0,"joint_setpoint":[0.1],"move_type":"relative"}
{"command_type":"stop"}
{"op":"state"}
```

Each line receives a JSON response. `op: state` returns the latest controller,
joints, commanded torques, errors and feedback age. This seven-joint SI-unit
protocol differs from Undroid's six-joint binary wire format.

## Validation and limits

```bash
colcon test --base-paths fer_* --packages-select fer_model fer_ctrl fer_driver \
  --return-code-on-test-failure
colcon test-result --verbose
ROS_DOMAIN_ID=171 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/test_simulation.py
```

Tests exercise task gains, enable flags, conflicting priorities, plane avoidance,
kinematics, state transitions, dynamic control laws, torque/rate limits, reference
shaping, ROS2 launch, TCP commands, and loss of either controller.

The simulator uses configurable diagonal joint inertia and viscous damping,
with gravity compensation assumed. It is a control-pipeline test model, without
full Panda dynamics, contacts, collision geometry or grasped objects.
Hardware uses the live libfranka mass, Coriolis, TCP and Jacobian data.
The geometric model and joint limits target Panda; FR3 is not supported by this
configuration. Physical robot operation has not been validated here.
