#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
            'cmd_vel_raw',
            self.listener_callback,
            10)
        
        # Create a publisher to the modified message for the controller
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_unstamped', 10)
        
    def listener_callback(self, msg):
        """
        Callback function for the subscriber. This function is called every time
        a new message is received on the subscribed topic.
        """
        self.get_logger().info(f'Received Twist message: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

        # Create a new Twist message for publishing
        modified_twist = Twist()
        
        # Perform the mathematical operation: scale the linear and angular components
        modified_twist.linear.x = msg.linear.x
        if math.abs(msg.angular.z) < 0.1:
            modified_twist.angular.z = 1000.0
        else:
            modified_twist.angular.z = -4.0 * math.atan(msg.angular.z - math.pi/2.0)
        
        # Publish the modified message
        self.publisher_.publish(modified_twist)
        
        self.get_logger().info(f'Published modified Twist: linear.x={modified_twist.linear.x}, angular.z={modified_twist.angular.z}')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of the TwistModifier node
    twist_modifier_node = TwistModifier()
    
    # Spin the node to process callbacks
    rclpy.spin(twist_modifier_node)
    
    # Destroy the node and shutdown rclpy when done
    twist_modifier_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

