### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
import rclpy.parameter
import smach
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        
        smach.State.__init__(self, outcomes=['low_battery', 'collision', 'safe'], 
                             output_keys=['battery_level', 'collision_detected'], 
                             input_keys=['battery_level', 'collision_detected'])
        
        self.node = node
        self.battery_level = None
        self.collision_detected = False
        self.battery_threshold = 50.0
        self.min_range_value = None
        
        self.collision_sub = self.node.create_subscription(LaserScan, '/scan', self.collision_callback, 10)
        self.battery_pub = self.node.create_publisher(Float32, '/battery_level', 10)
#         self.battery_sub = self.node.create_subscription(Float32, '/battery_level', self.battery_callback, 10)
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # raise NotImplementedError()

#     def battery_callback(self, msg):
#         self.battery_level = msg.data # Getting the battery value of battery from here
    
    def collision_callback(self, msg):
        # Check if any laser scan reading is below a certain threshold, indicating a potential collision
        self.node.get_logger().info('Collision_Callback Received')
        self.min_range_value = min(msg.ranges)
        if min(msg.ranges) < 0.5:
            self.collision_detected = True
        else:
            self.collision_detected = False
    def execute(self, userdata):
        rclpy.spin_once(self.node, timeout_sec=1.0)
        # Moving the robot forward a bit by bit, so that it collides at some point
        twist = Twist()
        twist.linear.x = 0.5
        self.cmd_pub.publish(twist)

        if self.battery_level is None:
            self.battery_level = 100
            userdata['battery_level'] = self.battery_level
        else:
            self.battery_level = userdata['battery_level']

        self.node.get_logger().info('Monitoring battery and collision...')
        self.node.get_logger().info(f'Battery level = {self.battery_level}')
        if self.battery_level > 60.0:
            self.node.get_logger().info('🔋🔋🔋')
        else:
            self.node.get_logger().info('🪫🪫🪫')
        self.node.get_logger().info(f'Min Range Value = {self.min_range_value}')
        self.battery_level -= 0.1  # Decrease battery level
        self.battery_pub.publish(Float32(data=self.battery_level))
        
        userdata['battery_level'] = self.battery_level
        userdata['collision_detected'] = self.collision_detected
        
        # Reset the collision_reset parameter to False after a manual reset
        if self.node.get_parameter('collision_reset').get_parameter_value().bool_value:
            self.node.set_parameters([rclpy.parameter.Parameter(name='collision_reset', value=False)])
        
        if self.battery_level < self.battery_threshold:
            return 'low_battery'
        elif self.collision_detected:
            return 'collision'
        else:
            return 'safe'
    



class RotateBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        
        self.battery_threshold = 99.0
        smach.State.__init__(self, outcomes=['battery_ok'], 
                             output_keys=['battery_level'],
                             input_keys=['battery_level'])
        self.node = node
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_pub = self.node.create_publisher(Float32, '/battery_level', 10)
        
        # raise NotImplementedError()

    def execute(self, userdata):
        
        self.node.get_logger().info('Rotating base due to low battery...')
        twist = Twist()
        twist.angular.z = 0.5
        
        while rclpy.ok() and userdata.battery_level < self.battery_threshold:
            userdata.battery_level += 0.1  # Increase battery level
            self.battery_pub.publish(Float32(data=userdata.battery_level))
            self.cmd_pub.publish(twist)
            self.node.get_logger().info(f'Battery level = {userdata.battery_level}')
            self.node.get_logger().info('🪫⚡🔌🔋')
            rclpy.spin_once(self.node, timeout_sec=1.0)
        
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        return 'battery_ok'
        
        # raise NotImplementedError()


class StopMotion(smach.State):
    """State to stop the robot's motion
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        self.battery_threshold = 50.0
        
        smach.State.__init__(self, outcomes=['manual_reset', 'low_battery'], 
                             output_keys=['battery_level'], 
                             input_keys=['battery_level'])
        self.node = node
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_pub = self.node.create_publisher(Float32, '/battery_level', 10)
        
        # raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        
        self.node.get_logger().info('Stopping motion due to potential collision...')
        twist = Twist()
        self.cmd_pub.publish(twist)
        # Run this command in terminal:( ros2 run teleop_twist_keyboard teleop_twist_keyboard )
        # Move the robot and run this command ( ros2 param set /AST_Assignment collision_reset True )
        # 
        while rclpy.ok() and not self.node.get_parameter('collision_reset').get_parameter_value().bool_value:
            userdata.battery_level -= 0.1  # Decrease battery level
            self.battery_pub.publish(Float32(data=userdata.battery_level))
            self.node.get_logger().info('Waiting for Manual Reset')
            self.node.get_logger().info(f'Battery level = {userdata.battery_level}')
            if userdata.battery_level > 60.0:
                self.node.get_logger().info('🔋🔋🔋')
            else:
                self.node.get_logger().info('🪫🪫🪫')

            if userdata.battery_level < self.battery_threshold:
                return 'low_battery'
            
            rclpy.spin_once(self.node, timeout_sec=1.0)
            # To get out of this loop, robot needs to be moved manually and then the parameter
            # needs 'collision_reset' needs to be set as True manually.
        return 'manual_reset'
        
        # raise NotImplementedError()


# TODO: define any additional states if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """

    # TODO: initialise a ROS2 node, set any threshold values, and define the state machine
    # YOUR CODE HERE
    
    rclpy.init(args=args)
    node = Node('AST_Assignment')

    node.declare_parameter('battery_level', 100.0)
    node.declare_parameter('collision_reset', False)

    sm = smach.StateMachine(outcomes=['END'])

    with sm:
        
        smach.StateMachine.add('MONITOR_BATTERY_AND_COLLISION', MonitorBatteryAndCollision(node),
                         transitions={'low_battery': 'ROTATE_BASE',
                                      'collision': 'STOP_MOTION',
                                      'safe': 'MONITOR_BATTERY_AND_COLLISION'})
        
        smach.StateMachine.add('ROTATE_BASE', RotateBase(node),
                         transitions={'battery_ok': 'MONITOR_BATTERY_AND_COLLISION'})
        
        smach.StateMachine.add('STOP_MOTION', StopMotion(node),
                         transitions={'manual_reset': 'MONITOR_BATTERY_AND_COLLISION',
                                      'low_battery': 'ROTATE_BASE'})

    outcome = sm.execute()

    node.destroy_node()
    rclpy.shutdown()
    
    # raise NotImplementedError()

if __name__ == "__main__":
    main()