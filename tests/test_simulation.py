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
from fer_msgs.msg import ControlData, GripperState, StateData
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
        self.state = self.control = self.gripper = self.marker = None
        self.subscriptions = [
            self.node.create_subscription(StateData, 'state/state_data',
                                          lambda msg: setattr(self, 'state', msg), qos_profile_sensor_data),
            self.node.create_subscription(ControlData, 'ctrl/ctrl_data',
                                          lambda msg: setattr(self, 'control', msg), 1),
            self.node.create_subscription(GripperState, 'gripper/state',
                                          lambda msg: setattr(self, 'gripper', msg), 1),
            self.node.create_subscription(Marker, 'visualization/arm',
                                          lambda msg: setattr(self, 'marker', msg), 1),
        ]

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

    def until(self, condition, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
            for process, log in zip(self.processes, self.logs):
                if process.poll() is not None:
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
            stream.write(b'{"command_type":"stop"}\n'); stream.flush()
            self.assertTrue(json.loads(stream.readline())['success'])
            stream.write(b'{"op":"state"}\n'); stream.flush()
            status = json.loads(stream.readline())
            self.assertTrue(status['success'])
            self.assertEqual(len(status['state']['joints']), 7)
            stream.write(b'{"unexpected":true}\n'); stream.flush()
            self.assertFalse(json.loads(stream.readline())['success'])

        self.assertTrue(self.command('disable').success)
        self.until(lambda: self.control.ctrl_state == 'disabled' and not self.state.control_active)
        self.assertTrue(all(math.isfinite(q) for q in self.state.joint_state.position))

    def test_driver_watchdog_requires_recovery(self):
        self.start(['ros2', 'run', 'fer_sim', 'simulator_node', '--ros-args',
                    '-r', f'__ns:={self.namespace}'])
        self.until(lambda: self.state is not None)
        publisher = self.node.create_publisher(ControlData, 'ctrl/ctrl_data', 1)
        message = ControlData()
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


if __name__ == '__main__':
    unittest.main(verbosity=2)
