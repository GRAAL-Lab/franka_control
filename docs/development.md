# Editing the control stack

Read the header before changing an implementation. Function contracts live in
Doxygen comments alongside declarations; keep units, reference frames, clock
assumptions and failure results accurate when behavior changes. Use the existing
`.clang-format` style. Keep command validation at the input boundary and keep
the control-cycle order explicit.

## Where to change behavior

| Change | Location |
| --- | --- |
| Task gains, limits, actions, priorities and park/unpark postures | `fer_ctrl/conf/kcl_fer.conf` |
| Dynamic gains, motion bounds and torque limits | `fer_ctrl/conf/dcl_fer.conf` |
| Global simulation selection and speed | `fer_bringup/launch/fer.launch.py`: `isSim`, `simTimeScale` |
| Simulation inertia, damping, initial posture and tool offset | `fer_sim/conf/simulator.yaml` |
| Robot geometry or the shared default tool offset | `fer_model/include/fer_model/franka_arm_model.hpp`, `fer_model/src/franka_arm_model.cpp` |
| A command's accepted values and goal semantics | `fer_ctrl/{include,src}/fer_ctrl/commands` and `states` |
| State actions and command permissions | `TaskPriorityController::ConfigureStateMachine` and `ConfigureStatePermissions` |
| A new task | `fer_ctrl/{include,src}/fer_ctrl/tasks` and `TaskPriorityController::CreateTasks` |
| ROS names and transport conversions | `fer_msgs/include/fer_msgs` |
| TCP translation and framing | `ArmCommandParser` and `FrankaTCPBridge` |

Use RML definitions directly: `Eigen::TransformationMatrix`,
`Eigen::RotationMatrix`, `Eigen::Vector6d` and `rml::EulerRPY`. The seven-joint
aliases are defined in the shared model header. `FrankaArmModel` is the single
Panda model and inherits `rml::ArmModel`.

Command service success means acceptance. Motion completion is reported through
controller state and feedback. Keep driver service requests asynchronous in the
controller, TCP bridge and visualizer so their control timers remain responsive.
Numeric TCP identifiers, packed field order and payload lengths are the wire ABI;
renaming an internal handler must not change that layout.

## Timing and threading

The simulator owns `/clock`. Its wall timer advances one fixed simulation step;
`simTimeScale` changes the wall period. `UpdateControlState` evaluates its
watchdog and lifecycle transitions, and `AdvanceArm` subdivides the physics step
into intervals of at most 1 ms. Both physics and gripper motion use simulated
seconds. Controller, visualizer and bridge timers use the ROS node clock.

`TaskPriorityController` receives time from its caller. With simulation enabled,
each `Run` or `ExecuteCommand` forwards seconds as milliseconds to TPIK's
`ActionManager::Time`. Hardware watchdogs use steady wall time. Keep these clock
domains explicit when adding a timeout.

ROS nodes use serialized callbacks except for the hardware gripper's reentrant
group. The arm driver owns its libfranka objects on one dedicated thread and
exchanges fixed-size records through single-producer/single-consumer
`LatestValue` buffers. Avoid ROS publication, logging or locks in the torque
callback.

## Verification without hardware

From the workspace root, with ROS Jazzy sourced:

```bash
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFER_BUILD_HARDWARE=OFF -DBUILD_TESTING=ON
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
source install/local_setup.bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 src/franka_control/tests/test_simulation.py
```

From the repository root, `doxygen Doxyfile` generates the header reference in
`docs/api/html`. Documentation generation reads headers without building or
connecting to hardware.
