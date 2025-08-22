import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_test', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Create a new Twist message for publishing
        vx = 0.25  # m/s
        turn_radius = 1.5  # m
        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.angular.z = vx / turn_radius

        self.publisher_.publish(twist_msg)
        self.get_logger().info('Publishing: "%s"' % twist_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()