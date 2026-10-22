#!/usr/bin/env python3
"""Run after sourcing the built workspace: python3 tests/test_simulation.py."""

import copy
import math
import os
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fer_msgs.msg import ControlData, DynamicControlData, GripperState, StateData, TPIKAction, TaskStatus
from fer_msgs.srv import ControlCommand, DriverCommand
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import InteractiveMarkerFeedback, Marker, MarkerArray
from visualization_msgs.srv import GetInteractiveMarkers
from tf2_msgs.msg import TFMessage


class SimulationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.namespace = f'/fer_test_{os.getpid()}_{self._testMethodName}'
        self.node = Node('test_client', namespace=self.namespace)
        self.processes = []
        self.logs = []
        self.state = None
        self.feedback_count = 0
        self.control = None
        self.gripper = None
        self.marker = None
        self.markers = None
        self.transforms = {}
        self.dynamic = None
        self.action = None
        self.joint_task = None
        self.expected_exits = set()
        self.subscriptions = [
            self.node.create_subscription(DynamicControlData, 'ctrl/dynamic_control', self.dynamic_callback, 1),
            self.node.create_subscription(TPIKAction, 'ctrl/tpik_action', self.action_callback, 1),
            self.node.create_subscription(TaskStatus, 'tasks/JointsPosition', self.task_callback, 1),
            self.node.create_subscription(StateData, 'state/state_data',
                                          self.state_callback, qos_profile_sensor_data),
            self.node.create_subscription(ControlData, 'ctrl/ctrl_data',
                                          self.control_callback, 1),
            self.node.create_subscription(GripperState, 'gripper/state',
                                          self.gripper_callback, 1),
            self.node.create_subscription(Marker, 'visualization/arm',
                                          self.marker_callback, 1),
            self.node.create_subscription(MarkerArray, 'visualization/joint_markers',
                                          self.markers_callback, 1),
            self.node.create_subscription(TFMessage, '/tf', self.tf_callback, 100),
        ]

    def state_callback(self, message):
        self.state = message
        self.feedback_count += 1

    def control_callback(self, message):
        self.control = message

    def gripper_callback(self, message):
        self.gripper = message

    def marker_callback(self, message):
        self.marker = message

    def markers_callback(self, message):
        self.markers = message

    def tf_callback(self, message):
        for transform in message.transforms:
            self.transforms[transform.child_frame_id] = transform

    def dynamic_callback(self, message):
        self.dynamic = message

    def action_callback(self, message):
        self.action = message

    def task_callback(self, message):
        self.joint_task = message

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process, log in zip(self.processes, self.logs):
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            if process.returncode not in (0, -signal.SIGINT) and process.pid not in self.expected_exits:
                log.seek(0)
                print(log.read())
            log.close()
        self.node.destroy_node()

    def start(self, command):
        log = tempfile.TemporaryFile(mode='w+')
        environment = dict(os.environ, RCUTILS_COLORIZED_OUTPUT='0')
        self.processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               env=environment, start_new_session=True))
        self.logs.append(log)
        return self.processes[-1]

    def until(self, condition, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
            for process, log in zip(self.processes, self.logs):
                if process.poll() is not None and process.pid not in self.expected_exits:
                    log.seek(0)
                    self.fail(f'Process exited with {process.returncode}:\n{log.read()}')
        self.fail('Timed out waiting for condition; control=' + str(self.control))

    def call(self, kind, name, **fields):
        client = self.node.create_client(kind, name)
        self.assertTrue(client.wait_for_service(timeout_sec=15.0), name)
        request = kind.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = client.call_async(request)
        self.until(future.done)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def command(self, command_type, **fields):
        return self.call(ControlCommand, 'srv/control_command', command_type=command_type, **fields)

    def test_complete_launch_and_tcp(self):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        self.start(['ros2', 'launch', 'fer_bringup', 'fer.launch.py',
                    f'namespace:={self.namespace}', 'with_tcp:=true', f'tcp_port:={port}'])
        self.start(['ros2', 'run', 'fer_sim', 'visualizer_node', '--ros-args',
                    '-r', f'__ns:={self.namespace}', '-p', 'isSim:=true'])
        self.until(lambda: self.state is not None and self.control is not None, timeout=20.0)
        self.assertEqual(self.control.ctrl_state, 'disabled')
        self.until(lambda: self.dynamic is not None and self.action is not None)
        self.assertEqual(len(self.state.joint_state.position), 7)
        self.until(lambda: self.marker is not None)
        self.assertEqual(len(self.marker.points), 9)
        self.assertEqual(self.marker.header.frame_id, self.state.header.frame_id)
        self.assertFalse(self.command('park').success)
        self.assertTrue(self.command('enable').success)
        self.until(lambda: self.control.ctrl_state == 'idle' and self.state.control_active)
        self.assertTrue(self.command('enable').success)

        self.assertFalse(self.command('move_joint_i_pos', joint_index=7, joint_setpoint=[0.1]).success)
        self.assertFalse(self.command('move_joints_pos', joint_setpoint=[0.0] * 6).success)
        initial = self.state.joint_state.position[0]
        self.assertTrue(self.command('move_joint_i_pos', joint_index=0, joint_setpoint=[0.08],
                                     move_type='relative').success)
        self.until(lambda: self.control.ctrl_state == 'move_joints_pos')
        self.until(lambda: self.action.id == 'MoveJointsPosition' and self.joint_task.in_current_action)
        self.assertIn('PL_Safety', [level.id for level in self.action.priority_levels])
        self.assertEqual(len(self.joint_task.reference_rate), 7)
        self.assertTrue(self.dynamic.control_enabled)
        self.until(lambda: max(abs(value) for value in self.state.commanded_torques) > 0.001)
        self.until(lambda: self.control.ctrl_state == 'idle')
        self.assertAlmostEqual(self.state.joint_state.position[0], initial + 0.08, delta=0.012)

        initial_z = self.state.tool_pose.position.z
        self.assertTrue(self.command('move_cartesian', move_type='relative', frame_type=1,
                                     target_position=[0.0, 0.0, 0.01]).success)
        self.until(lambda: self.control.ctrl_state == 'move_cartesian')
        self.until(lambda: self.control.ctrl_state == 'idle')
        self.assertAlmostEqual(self.state.tool_pose.position.z, initial_z + 0.01, delta=0.0025)

        self.assertTrue(self.command('move_joint_i_vel', joint_index=0, joint_setpoint=[0.1]).success)
        self.until(lambda: self.control.ctrl_state == 'move_joints_vel')
        self.until(lambda: self.control.ctrl_state == 'idle')
        self.until(lambda: max(abs(v) for v in self.state.joint_state.velocity) < 0.005)

        self.assertTrue(self.command('gripper_move', gripper_setpoint=0.04).success)
        self.until(lambda: self.gripper is not None and abs(self.gripper.width - 0.04) < 0.001)

        self.assertTrue(self.command('park').success)
        self.until(lambda: self.control.ctrl_state == 'parking')
        self.assertFalse(self.command('move_joint_i_pos', joint_index=0, joint_setpoint=[0.1]).success)
        self.assertFalse(self.command('move_joint_i_vel', joint_index=0, joint_setpoint=[0.1]).success)
        self.assertFalse(self.command('move_cartesian', move_type='relative', frame_type=1,
                                      target_position=[0.0, 0.0, 0.01]).success)
        self.until(lambda: self.control.ctrl_state == 'parked', timeout=20.0)
        self.assertFalse(self.command('move_joint_i_vel', joint_setpoint=[0.1]).success)

        # The simulated driver keeps Undroid's joint-position exception in parked.
        self.assertTrue(self.command('move_joint_i_pos', joint_index=0, joint_setpoint=[0.02],
                                     move_type='relative').success)
        self.until(lambda: self.control.ctrl_state == 'move_joints_pos')
        self.until(lambda: self.control.ctrl_state == 'idle')
        self.assertTrue(self.command('park').success)
        self.until(lambda: self.control.ctrl_state == 'parked', timeout=20.0)

        self.assertTrue(self.command('unpark').success)
        self.until(lambda: self.control.ctrl_state == 'unparking')
        self.assertFalse(self.command('move_joint_i_pos', joint_index=0, joint_setpoint=[0.1]).success)
        self.assertFalse(self.command('park').success)
        self.until(lambda: self.control.ctrl_state == 'idle', timeout=20.0)

        with socket.create_connection(('127.0.0.1', port), timeout=3.0) as connection:
            stream = connection.makefile('rb')

            def read_report(report_id):
                for _ in range(1000):
                    header = stream.read(12)
                    self.assertEqual(len(header), 12)
                    _, received_id, length = struct.unpack('<III', header)
                    self.assertLessEqual(length, 64)
                    payload = stream.read(length)
                    self.assertEqual(len(payload), length)
                    if received_id == report_id:
                        return payload
                self.fail('Expected TCP report was not received')

            # Fragment a command header across writes to exercise TCP stream framing.
            stop = struct.pack('<III', 0, 0, 0)
            connection.sendall(stop[:5])
            time.sleep(0.02)
            connection.sendall(stop[5:])
            joints = struct.unpack('<8d', read_report(1))
            self.assertEqual(len(joints[:7]), 7)
            self.assertTrue(all(math.isfinite(value) for value in joints))
            self.assertEqual(len(read_report(2)), 6 * 8)
            self.assertEqual(len(read_report(3)), 7 * 8)

            # A binary relative command can address the seventh Panda joint.
            initial_joint_7 = self.state.joint_state.position[6]
            payload = struct.pack('<id', 6, 0.02)
            connection.sendall(struct.pack('<III', 0, 11, len(payload)) + payload)
            self.until(lambda: self.control.ctrl_state == 'move_joints_pos')
            self.until(lambda: self.control.ctrl_state == 'idle')
            self.assertAlmostEqual(self.state.joint_state.position[6], initial_joint_7 + 0.02, delta=0.012)

            # Reject a six-joint payload and keep the next coalesced frame aligned.
            invalid = struct.pack('<III6d', 0, 9, 48, *([0.0] * 6))
            connection.sendall(invalid + stop)
            self.assertEqual(len(read_report(0)), 4)
            stream.close()

        client = self.start(['ros2', 'run', 'fer_tcp_bridge', 'dummy_client_node', '--ros-args',
                             '-p', f'port:={port}', '-p', 'max_reports:=3'])
        self.expected_exits.add(client.pid)
        self.until(lambda: client.poll() is not None)
        self.assertEqual(client.returncode, 0)

        self.assertTrue(self.command('disable').success)
        self.until(lambda: self.control.ctrl_state == 'disabled' and not self.state.control_active)
        self.assertTrue(all(math.isfinite(q) for q in self.state.joint_state.position))

    def test_global_simulation_clock_accelerates_physics_and_controllers(self):
        self.start(['ros2', 'launch', 'fer_bringup', 'fer.launch.py',
                    f'namespace:={self.namespace}', 'isSim:=true', 'simTimeScale:=2.0'])
        self.until(lambda: self.state is not None and self.control is not None
                   and self.dynamic is not None and self.gripper is not None, timeout=20.0)
        for node_name in ('kinematic_control_node', 'dynamic_control_node', 'simulator_node'):
            parameters = self.call(GetParameters, node_name + '/get_parameters',
                                   names=['isSim', 'use_sim_time'])
            self.assertTrue(all(value.bool_value for value in parameters.values))
        parameters = self.call(GetParameters, 'simulator_node/get_parameters', names=['simTimeScale'])
        self.assertEqual(parameters.values[0].double_value, 2.0)

        self.assertTrue(self.call(DriverCommand, 'gripper/command', command_type='move',
                                 width=0.0, speed=0.02, force=20.0).success)
        previous = self.gripper
        self.until(lambda: self.gripper is not previous)
        initial = self.gripper
        simulated_start = initial.header.stamp.sec + initial.header.stamp.nanosec * 1e-9
        wall_start = time.monotonic()
        self.until(lambda: self.gripper.width <= initial.width - 0.02)
        wall_elapsed = time.monotonic() - wall_start
        simulated_elapsed = (self.gripper.header.stamp.sec
                             + self.gripper.header.stamp.nanosec * 1e-9 - simulated_start)
        self.assertAlmostEqual(initial.width - self.gripper.width, 0.02 * simulated_elapsed, delta=0.0002)
        self.assertGreater(simulated_elapsed / wall_elapsed, 1.4)
        self.assertLess(simulated_elapsed / wall_elapsed, 2.6)

        self.assertTrue(self.command('enable').success)
        self.until(lambda: self.state.control_active and self.control.ctrl_state == 'idle')
        initial_joint = self.state.joint_state.position[0]
        self.assertTrue(self.command('move_joint_i_vel', joint_index=0, joint_setpoint=[0.1]).success)
        self.until(lambda: self.control.ctrl_state == 'move_joints_vel')
        self.until(lambda: self.control.ctrl_state == 'idle')
        self.assertGreater(self.state.joint_state.position[0], initial_joint + 0.001)
        self.assertFalse(self.state.has_error)

    def test_invalid_simulation_scale_is_rejected(self):
        for scale in ('0.0', '-1.0'):
            process = self.start(['ros2', 'run', 'fer_sim', 'simulator_node', '--ros-args',
                                  '-r', f'__ns:={self.namespace}', '-p', f'simTimeScale:={scale}'])
            self.expected_exits.add(process.pid)
            self.until(lambda: process.poll() is not None)
            self.assertNotEqual(process.returncode, 0)
            self.logs[-1].seek(0)
            self.assertIn('simTimeScale must be finite and positive', self.logs[-1].read())

    def test_driver_watchdog_requires_recovery(self):
        self.start(['ros2', 'run', 'fer_sim', 'simulator_node', '--ros-args',
                    '-r', f'__ns:={self.namespace}', '-p', 'simTimeScale:=2.0'])
        self.until(lambda: self.state is not None)
        publisher = self.node.create_publisher(DynamicControlData, 'ctrl/dynamic_control', 1)
        message = DynamicControlData()
        message.header.frame_id = 'panda_link0'
        message.joint_positions = list(self.state.joint_state.position)
        message.control_enabled = True
        message.joint_velocities = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        timer = self.node.create_timer(0.02, lambda: publisher.publish(message))
        result = self.call(DriverCommand, 'driver/command', command_type='enable')
        self.assertTrue(result.success)
        self.until(lambda: self.state.control_active)
        self.node.destroy_timer(timer)
        before = self.state.header.stamp.sec + self.state.header.stamp.nanosec * 1e-9
        wall_start = time.monotonic()
        self.until(lambda: self.state.has_error)
        elapsed = self.state.header.stamp.sec + self.state.header.stamp.nanosec * 1e-9 - before
        self.assertGreater(elapsed, 0.15)
        self.assertLess(elapsed, 0.30)
        self.assertLess(time.monotonic() - wall_start, 0.25)
        self.until(lambda: not self.state.control_active)
        self.assertIn('watchdog', self.state.error)
        self.assertFalse(self.call(DriverCommand, 'driver/command', command_type='enable').success)
        self.assertTrue(self.call(DriverCommand, 'driver/command', command_type='recover').success)
        self.until(lambda: not self.state.has_error)
        self.assertFalse(self.state.control_active)

    def start_control_stack(self):
        simulator = self.start(['ros2', 'run', 'fer_sim', 'simulator_node', '--ros-args',
                                '-r', f'__ns:={self.namespace}'])
        dynamic = self.start(['ros2', 'run', 'fer_ctrl', 'dynamic_control_node', '--ros-args',
                              '-r', f'__ns:={self.namespace}', '-p', 'isSim:=true'])
        kinematic = self.start(['ros2', 'run', 'fer_ctrl', 'kinematic_control_node', '--ros-args',
                                '-r', f'__ns:={self.namespace}', '-p', 'isSim:=true'])
        self.until(lambda: self.state is not None and self.control is not None
                   and self.dynamic is not None and self.feedback_count >= 20, timeout=20.0)
        deadline = time.monotonic() + 5.0
        while True:
            response = self.command('enable')
            if response.success:
                break
            self.assertIn(response.res, ['Arm driver service is unavailable', 'No fresh robot feedback'])
            self.assertLess(time.monotonic(), deadline, response.res)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.until(lambda: self.state.control_active and self.control.ctrl_state == 'idle')
        return simulator, dynamic, kinematic

    def stop_process(self, process):
        self.expected_exits.add(process.pid)
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8.0)

    def test_kinematic_loss_disables_dynamic_references(self):
        _, _, kinematic = self.start_control_stack()
        self.assertTrue(self.command('move_joint_i_vel', joint_index=0, joint_setpoint=[0.1]).success)
        self.until(lambda: self.control.ctrl_state == 'move_joints_vel')
        self.stop_process(kinematic)
        self.until(lambda: not self.dynamic.control_enabled and not self.state.control_active)
        self.assertIn('timeout', self.dynamic.status)
        self.assertLess(max(abs(value) for value in self.state.joint_state.velocity), 0.01)

    def test_dynamic_loss_requires_recovery_and_explicit_enable(self):
        _, dynamic, _ = self.start_control_stack()
        self.stop_process(dynamic)
        self.until(lambda: self.state.has_error and not self.state.control_active)
        self.until(lambda: self.control.ctrl_state == 'fault')
        self.assertIn('watchdog', self.state.error)
        self.assertFalse(self.command('enable').success)
        self.assertTrue(self.command('recover').success)
        self.until(lambda: not self.state.has_error and self.control.ctrl_state == 'disabled')
        self.assertFalse(self.state.control_active)

    def test_interactive_tool_control_and_goal_tf(self):
        self.start_control_stack()
        self.start(['ros2', 'run', 'fer_sim', 'visualizer_node', '--ros-args',
                    '-r', f'__ns:={self.namespace}', '-p', 'isSim:=true'])
        self.until(lambda: self.markers is not None
                   and all(f'panda_joint{joint}' in self.transforms for joint in range(1, 8)))
        joints = [marker for marker in self.markers.markers if marker.ns == 'manipulator_joints']
        self.assertEqual(len(joints), 7)
        self.assertTrue(any(marker.ns == 'manipulator_base' for marker in self.markers.markers))
        planes = [marker for marker in self.markers.markers if marker.ns == 'avoidance_planes']
        self.assertEqual(len(planes), 1)
        self.assertAlmostEqual(planes[0].pose.position.z, 0.1)
        for joint in range(1, 8):
            self.assertEqual(self.transforms[f'panda_joint{joint}'].header.frame_id, 'panda_link0')

        response = self.call(GetInteractiveMarkers, 'tool_control/get_interactive_markers')
        self.assertEqual(len(response.markers), 1)
        marker = response.markers[0]
        self.assertEqual(marker.name, 'tool_control')
        self.assertEqual(len(marker.controls), 7)
        self.assertAlmostEqual(marker.pose.position.x, self.state.tool_pose.position.x, delta=0.001)

        publisher = self.node.create_publisher(InteractiveMarkerFeedback, 'tool_control/feedback', 10)
        self.until(lambda: publisher.get_subscription_count() > 0)
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = 'panda_link0'
        feedback.client_id = 'integration_test'
        feedback.marker_name = marker.name
        feedback.control_name = 'move_x'
        feedback.pose = copy.deepcopy(marker.pose)
        feedback.event_type = InteractiveMarkerFeedback.MOUSE_DOWN
        publisher.publish(feedback)
        feedback.pose.position.x += 0.015
        feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        publisher.publish(feedback)
        self.until(lambda: 'ctrl_frame' in self.transforms
                   and abs(self.transforms['ctrl_frame'].transform.translation.x - feedback.pose.position.x) < 1e-8)
        self.assertEqual(self.control.ctrl_state, 'idle')

        feedback.event_type = InteractiveMarkerFeedback.MOUSE_UP
        publisher.publish(feedback)
        self.until(lambda: self.control.ctrl_state == 'move_cartesian')
        self.until(lambda: 'goal' in self.transforms
                   and abs(self.transforms['goal'].transform.translation.x - feedback.pose.position.x) < 1e-8)
        goal = self.transforms['goal']
        self.assertEqual(goal.header.frame_id, 'panda_link0')
        self.assertAlmostEqual(goal.transform.translation.y, feedback.pose.position.y)
        self.assertAlmostEqual(goal.transform.translation.z, feedback.pose.position.z)
        orientation = goal.transform.rotation
        expected = feedback.pose.orientation
        quaternion_dot = (orientation.x * expected.x + orientation.y * expected.y
                          + orientation.z * expected.z + orientation.w * expected.w)
        self.assertAlmostEqual(abs(quaternion_dot), 1.0)
        self.until(lambda: self.control.ctrl_state == 'idle')
        self.assertAlmostEqual(self.state.tool_pose.position.x, feedback.pose.position.x, delta=0.0025)


if __name__ == '__main__':
    unittest.main(verbosity=2)
