#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
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
        
        # Create publishers for stamped and unstamped messages
        self.publisher_stamped = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.publisher_unstamped = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

    def listener_callback(self, msg):
        """
        Callback function for the subscriber. This function is called every time
        a new message is received on the subscribed topic.
        """
        #self.get_logger().info(f'Received Twist message: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

        # Create a new TwistStamped message for publishing
        modified_twist = TwistStamped()
        modified_twist.header.stamp = self.get_clock().now().to_msg()
        modified_twist.header.frame_id = "base_link"

        # Perform the mathematical operation: scale the linear and angular components
        modified_twist.twist.linear.x = msg.linear.x
        turn_radius = 0.0
        if math.fabs(msg.angular.z) < 0.1:
            turn_radius = 1000.0
        else:
            turn_radius = -2.0 * math.tan(msg.angular.z - math.pi/2.0)

        modified_twist.twist.angular.z = msg.linear.x / turn_radius if turn_radius != 0 else 0.0

        # Publish the stamped message
        self.publisher_stamped.publish(modified_twist)
        
        # Also publish the unstamped Twist message
        unstamped_twist = Twist()
        unstamped_twist.linear.x = modified_twist.twist.linear.x
        unstamped_twist.linear.y = modified_twist.twist.linear.y
        unstamped_twist.linear.z = modified_twist.twist.linear.z
        unstamped_twist.angular.x = modified_twist.twist.angular.x
        unstamped_twist.angular.y = modified_twist.twist.angular.y
        unstamped_twist.angular.z = modified_twist.twist.angular.z
        self.publisher_unstamped.publish(unstamped_twist)
        
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

