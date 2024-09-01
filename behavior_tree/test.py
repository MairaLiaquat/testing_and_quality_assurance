import unittest
from unittest.mock import MagicMock
from state_machine1 import MonitorBatteryAndCollision, RotateBase, StopMotion

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
        # Mock out rclpy functions that could cause issues
        with unittest.mock.patch('rclpy.spin_once'):
            outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'low_battery')

    def test_execute_collision(self):
        userdata = {'collision_detected': True}
        self.state.collision_detected = userdata['collision_detected']
        with unittest.mock.patch('rclpy.spin_once'):
            outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'collision')

    def test_execute_safe(self):
        userdata = {'battery_level': 80, 'collision_detected': False}
        self.state.battery_level = userdata['battery_level']
        self.state.collision_detected = userdata['collision_detected']
        with unittest.mock.patch('rclpy.spin_once'):
            outcome = self.state.execute(userdata)
        self.assertEqual(outcome, 'safe')

# Similarly adjust TestRotateBase and TestStopMotion as necessary

if __name__ == '__main__':
    unittest.main(verbosity=2)
