#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zoe2_interfaces.msg import DriveCmd
import math

class TwistModifier(Node):
    """
    A simple ROS2 node that subscribes to the Twist from joystick, modifies the
    angular velocity to drive radius, and then publishes the new message.
    """

    def __init__(self):
        super().__init__('joystick_twist_modifier')
        
        # Create a subscriber to the topic published by the joystick
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.listener_callback,
            10)
        
        # Create a publisher to the modified message for the controller
        self.publisher_ = self.create_publisher(DriveCmd, '/drive_cmd_unstamped', 10)

        self.declare_parameter('max_joy_speed', 1.0)
        self.declare_parameter('max_joy_turning_angle', math.radians(25))

    def listener_callback(self, msg):
        """
        Callback function for the subscriber. This function is called every time
        a new message is received on the subscribed topic.
        """
        # self.get_logger().info(f'Received Twist message: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

        # Create a new DriveCmd message for publishing
        drive_cmd_out = DriveCmd()

        # Perform the mathematical operation: scale the linear and angular components
        drive_cmd_out.speed = msg.linear.x * self.get_parameter('max_joy_speed').value  # speed in m/s
        drive_cmd_out.angle = msg.angular.z * self.get_parameter('max_joy_turning_angle').value  # angle in radians

        # Publish the modified message
        self.publisher_.publish(drive_cmd_out)
        
        #self.get_logger().info(f'Published modified Twist: linear.x={modified_twist.linear.x}, angular.z={modified_twist.angular.z}')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of the TwistModifier node
    twist_modifier_node = TwistModifier()
    
    try:
        # Spin the node to process callbacks
        rclpy.spin(twist_modifier_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node and shutdown rclpy when done
        twist_modifier_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

