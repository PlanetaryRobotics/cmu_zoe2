import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_vx = 0.0
        self.vx_increment = 0.001  # m/s per timer tick
        self.max_vx = 1.0  # m/s

    def timer_callback(self):
        # Ramp up vx linearly until max_vx
        turn_radius = 1.5  # m
        twist_msg = Twist()
        twist_msg.linear.x = self.current_vx
        twist_msg.angular.z = self.current_vx / turn_radius

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Publishing: vx={self.current_vx:.2f}, angular.z={twist_msg.angular.z:.2f}')

        if self.current_vx < self.max_vx:
            self.current_vx += self.vx_increment
            if self.current_vx > self.max_vx:
                self.current_vx = self.max_vx


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