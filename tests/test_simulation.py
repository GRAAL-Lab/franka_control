#!/usr/bin/env python3
"""Run after sourcing the built workspace: python3 tests/test_simulation.py."""

import json
import math
import os
import signal
import socket
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fer_msgs.msg import ControlData, DynamicControlData, GripperState, StateData, TPIKAction, TaskStatus
from fer_msgs.srv import ControlCommand, DriverCommand
from visualization_msgs.msg import Marker


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
        self.control = None
        self.gripper = None
        self.marker = None
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
        ]

    def state_callback(self, message):
        self.state = message

    def control_callback(self, message):
        self.control = message

    def gripper_callback(self, message):
        self.gripper = message

    def marker_callback(self, message):
        self.marker = message

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
            if process.returncode not in (0, -signal.SIGINT):
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
                    '-r', f'__ns:={self.namespace}'])
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
        self.until(lambda: self.control.ctrl_state == 'parked', timeout=20.0)
        self.assertFalse(self.command('move_joint_i_vel', joint_setpoint=[0.1]).success)
        self.assertTrue(self.command('unpark').success)
        self.until(lambda: self.control.ctrl_state == 'idle', timeout=20.0)

        with socket.create_connection(('127.0.0.1', port), timeout=3.0) as connection:
            stream = connection.makefile('rwb')
            stream.write(b'{"command_type":"stop"}\n')
            stream.flush()
            self.assertTrue(json.loads(stream.readline())['success'])
            stream.write(b'{"op":"state"}\n')
            stream.flush()
            status = json.loads(stream.readline())
            self.assertTrue(status['success'])
            self.assertEqual(len(status['state']['joints']), 7)
            stream.write(b'{"unexpected":true}\n')
            stream.flush()
            self.assertFalse(json.loads(stream.readline())['success'])

        self.assertTrue(self.command('disable').success)
        self.until(lambda: self.control.ctrl_state == 'disabled' and not self.state.control_active)
        self.assertTrue(all(math.isfinite(q) for q in self.state.joint_state.position))

    def test_driver_watchdog_requires_recovery(self):
        self.start(['ros2', 'run', 'fer_sim', 'simulator_node', '--ros-args',
                    '-r', f'__ns:={self.namespace}'])
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
        self.until(lambda: self.state.has_error and not self.state.control_active)
        self.assertIn('watchdog', self.state.error)
        self.assertFalse(self.call(DriverCommand, 'driver/command', command_type='enable').success)
        self.assertTrue(self.call(DriverCommand, 'driver/command', command_type='recover').success)
        self.until(lambda: not self.state.has_error)
        self.assertFalse(self.state.control_active)

    def start_control_stack(self):
        simulator = self.start(['ros2', 'run', 'fer_sim', 'simulator_node', '--ros-args',
                                '-r', f'__ns:={self.namespace}'])
        dynamic = self.start(['ros2', 'run', 'fer_ctrl', 'dynamic_control_node', '--ros-args',
                              '-r', f'__ns:={self.namespace}'])
        kinematic = self.start(['ros2', 'run', 'fer_ctrl', 'kinematic_control_node', '--ros-args',
                                '-r', f'__ns:={self.namespace}'])
        self.until(lambda: self.state is not None and self.control is not None
                   and self.dynamic is not None, timeout=20.0)
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
        self.until(lambda: not self.state.has_error)
        self.assertEqual(self.control.ctrl_state, 'disabled')
        self.assertFalse(self.state.control_active)


if __name__ == '__main__':
    unittest.main(verbosity=2)
