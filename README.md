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
| `fer_model` | `FrankaArmModel`, the RML arm model shared by the controller, simulator and visualizer |
| `fer_ctrl` | Kinematic controller, command/state classes, configuration loaders and dynamic controller |
| `fer_driver` | libfranka arm torque driver and independent gripper node |
| `fer_sim` | Joint-dynamics simulator, RViz geometry, joint TFs and interactive tool control |
| `fer_tcp_bridge` | C++ binary ArmProtocol bridge, separate parser, feedback loops and dummy client |
| `fer_bringup` | Complete hardware/simulation launch and RViz configuration |

Headers and implementations are separated into `include/<package>/` and
`src/<package>/`. Commands and states follow Undroid's `GenericCommand` and
`GenericState` organization. `InitializeROSInterfaces` creates node interfaces,
`Handle...` methods process messages or services, `Run` advances one cycle,
and `Publish...` methods publish results. The core creates tasks in `CreateTasks`
and wires state dependencies and permissions in `ConfigureStateMachine`.
Executable source files contain the ROS initialization and spin loop.
[.clang-format](.clang-format) records the common C++ style.

Headers document function behavior, units, frames, timing and failure results
using Doxygen comments. Generate the API reference from this repository root:

```bash
doxygen Doxyfile
# Open docs/api/html/index.html
```

See [the editing guide](docs/development.md) for the main extension points.

See [the architecture](docs/architecture.md) for the state machine, controller
equations, ROS interfaces, and realtime boundary.

## Dependencies and build

Install ROS2 Jazzy and colcon, including the ROS packages `interactive_markers`,
`rviz2`, and `eigen3_cmake_module`. Install or update the system-wide GRAAL libraries
with the [GRAAL dependency installer](https://github.com/GRAAL-Lab/graal_utils/blob/main/scripts/install_update_graal_libs.sh)
from a checkout of `graal_utils`:

```bash
bash /path/to/graal_utils/scripts/install_update_graal_libs.sh
```

The script builds and installs `rml`, `fsm`, `ctrl_toolbox`, `tpik`, and `ikcl`
in dependency order, along with other GRAAL libraries and system dependencies,
including Eigen3 and `libconfig++-dev`. Their CMake packages must be discoverable
through the normal installation prefixes or `CMAKE_PREFIX_PATH`.

For hardware, install libfranka separately, choosing a release compatible with
the Panda firmware. This driver's build was verified with
[libfranka 0.9.2](https://github.com/frankarobotics/libfranka/releases/tag/0.9.2).
Use the [0.9.2 API headers](https://github.com/frankarobotics/libfranka/tree/0.9.2/include/franka)
and [0.9.2 examples](https://github.com/frankarobotics/libfranka/tree/0.9.2/examples)
as the driver API references.

The driver links `Franka::Franka`. CMake requests a minimum version of 0.9 through
`find_package(Franka 0.9 REQUIRED)`; 0.9.2 is the build-verified baseline.
Other releases, including 0.15.0, have not been verified for this stack.

The upstream [realtime host setup](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)
is current Linux kernel and scheduling guidance, independent of the driver API
version. Use it for host configuration; use the release-pinned references above
for libfranka API details.

Run from this repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

For a custom libfranka installation, append
`-DFranka_DIR=/path/to/lib/cmake/Franka` to the CMake arguments.

To build for simulation without libfranka:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFER_BUILD_HARDWARE=OFF
source install/setup.bash
```

The GRAAL controller libraries are required in both builds. Source the installed
workspace in each terminal. Tests are disabled by default. If an existing build
directory has testing enabled, add `-DBUILD_TESTING=OFF` to the CMake arguments
once to update its cached setting.

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

Bringup controls simulation globally with `isSim` (default `true`) and
`simTimeScale` (default `1.0`):

```bash
ros2 launch fer_bringup fer.launch.py isSim:=true simTimeScale:=2.0
```

The simulator publishes the shared ROS `/clock`. Physics, gripper motion,
controller updates, action transitions, watchdogs and message timestamps follow
that clock. `simTimeScale:=2.0` targets twice real-time speed by scheduling fixed
numerical steps more frequently. The integration step stays the same; achieved
speed depends on available CPU. The scale must be finite and positive.
`isSim:=false` selects the hardware backend and system time. For nodes started
individually, pass `-p isSim:=true` to the controllers and visualizer to follow
the simulator's clock. The simulator itself defaults to simulation mode.

The simulator's `tool_offset` parameter sets the flange-to-tool distance in metres.
It defaults to `fer::DEFAULT_TOOL_OFFSET` in `fer_model/franka_arm_model.hpp`; set
`tool_offset` in the simulator YAML to override it. All `FrankaArmModel` instances
use this shared default. The controller takes the actual tool
transform from driver or simulator feedback.

Hardware:

```bash
ros2 launch fer_bringup fer.launch.py isSim:=false robot_ip:=172.16.0.2
# Without the Franka hand:
ros2 launch fer_bringup fer.launch.py isSim:=false robot_ip:=172.16.0.2 with_gripper:=false
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
The C++ bridge follows Undroid's protocol/parser/command-loop/feedback-loop
organization. Its installed [armprotocol.h](fer_tcp_bridge/include/fer_tcp_bridge/armprotocol.h)
defines command IDs, reports and packed structures. Each packet contains three
little-endian `uint32` fields (`destination`, `msgId`, payload `length`), followed
by the payload. Length excludes the 12-byte header. Joint positions and efforts
contain seven values; six-joint commands are rejected.

This sends a stop command and reads the binary report stream:

```bash
ros2 run fer_tcp_bridge dummy_client_node --ros-args \
  -p port:=9999 -p command:=stop -p max_reports:=10
```

Leaving `command` empty makes the dummy client monitor feedback. The bridge
streams controller status, seven joint positions plus gripper opening, measured
tool pose, joint efforts and gripper grasp state. Command-service results are
logged; completion is observed through the report stream. Angles use radians,
positions use metres, gripper opening uses percent, and grasp force uses newtons.
The original `MOTORS_CURRENT` report ID now carries Franka joint torque in Nm.

## RViz tool control

Launch with `rviz:=true` to display the arm, seven joint markers, the base and
the avoidance planes from the selected `kcl_config`. The interactive tool marker
starts at the measured tool pose. Drag its translation or rotation handles and
release to submit an absolute BASE Cartesian goal through `ControlCommand`.
Feedback must be fresh and control enabled for motion to be accepted.

The default **Interact** tool supports both marker handles and camera navigation.
Drag empty space with the left mouse button to orbit, use the wheel to zoom,
and middle-drag or Shift+left-drag to pan. Select **Move Camera** to navigate
even when the pointer is over a marker. Camera navigation does not require
the robot controller to be enabled.

The visualizer publishes `panda_joint1` through `panda_joint7` TFs and a
`ctrl_frame` preview while dragging. During Cartesian motion, the kinematic
controller publishes the configured `goal_frame` (default `goal`) alongside the
measured tool frame. RViz topics are relative to the selected ROS namespace.

## Validation and limits

To build and run the tests, install `ament_cmake_gtest` and enable testing
explicitly:

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon test --return-code-on-test-failure
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
