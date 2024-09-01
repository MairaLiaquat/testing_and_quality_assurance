
import unittest
from unittest.mock import MagicMock
from .state_machine1 import MonitorBatteryAndCollision, RotateBase, StopMotion

class TestMonitorBatteryAndCollision(unittest.TestCase):
    def setUp(self):
        self.node = MagicMock()
        self.state = MonitorBatteryAndCollision(self.node)

    def test_initialization(self):
        self.assertEqual(self.state.battery_level, None)
        self.assertFalse(self.state.collision_detected)

    def test_collision_callback(self):
        msg = MagicMock()
        msg.ranges = [0.6, 0.4, 0.8]
        self.state.collision_callback(msg)
        self.assertTrue(self.state.collision_detected)
        
        msg.ranges = [0.6, 0.7, 0.8]
        self.state.collision_callback(msg)
        self.assertFalse(self.state.collision_detected)
        
    def test_execute_low_battery(self):
        userdata = {'battery_level': 49}
        self.state.battery_level = userdata['battery_level']
        outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'low_battery')

    def test_execute_collision(self):
        userdata = {'collision_detected': True}
        self.state.collision_detected = userdata['collision_detected']
        outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'collision')

    def test_execute_safe(self):
        userdata = {'battery_level': 80, 'collision_detected': False}
        self.state.battery_level = userdata['battery_level']
        self.state.collision_detected = userdata['collision_detected']
        outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'safe')


class TestRotateBase(unittest.TestCase):
    def setUp(self):
        self.node = MagicMock()
        self.state = RotateBase(self.node)

    def test_initialization(self):
        self.assertEqual(self.state.battery_threshold, 99.0)

    def test_execute_battery_ok(self):
        userdata = {'battery_level': 80}
        self.node.create_publisher().publish = MagicMock()
        outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'battery_ok')
        self.assertGreaterEqual(userdata['battery_level'], 99.0)


class TestStopMotion(unittest.TestCase):
    def setUp(self):
        self.node = MagicMock()
        self.node.get_parameter.return_value.get_parameter_value.return_value.bool_value = False
        self.state = StopMotion(self.node)

    def test_initialization(self):
        self.assertEqual(self.state.battery_threshold, 50.0)

    def test_execute_manual_reset(self):
        userdata = {'battery_level': 80}
        self.node.get_parameter().get_parameter_value().bool_value = True
        outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'manual_reset')

    def test_execute_low_battery(self):
        userdata = {'battery_level': 29}
        outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'low_battery')

    def test_execute_waiting_for_manual_reset(self):
        userdata = {'battery_level': 80}
        self.node.get_parameter().get_parameter_value().bool_value = False
        self.node.create_publisher().publish = MagicMock()

        # Run execute in a separate thread to simulate the while loop waiting
        import threading
        result = []

        def execute_state():
            outcome = self.state.execute(userdata)
            result.append(outcome)

        thread = threading.Thread(target=execute_state)
        thread.start()

        # Let the loop run for a short time and then simulate a manual reset
        import time
        time.sleep(1)
        self.node.get_parameter().get_parameter_value().bool_value = True
        thread.join(timeout=2)

        self.assertEqual(result[0], 'manual_reset')

if __name__ == '__main__':
    unittest.main()
