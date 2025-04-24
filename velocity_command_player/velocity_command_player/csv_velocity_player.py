import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import time
import os

class CSVVelocityPlayer(Node):
    def __init__(self, csv_path):
        super().__init__('csv_velocity_player')
        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.command_list = self.load_csv(csv_path)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_index = 0
        self.timer = self.create_timer(0.01, self.timer_callback)

    def load_csv(self, path):
        commands = []
        with open(path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                timestamp = float(row['timestamp'])
                speeds = [
                    float(row['wheel_front_left']),
                    float(row['wheel_front_right']),
                    float(row['wheel_back_left']),
                    float(row['wheel_back_right'])
                ]
                commands.append((timestamp, speeds))
        return commands

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        if self.current_index < len(self.command_list):
            ts, speeds = self.command_list[self.current_index]
            if now >= ts:
                msg = Float64MultiArray()
                msg.data = speeds
                self.publisher.publish(msg)
                self.get_logger().info(f'[{now:.2f}s] Published: {speeds}')
                self.current_index += 1
        else:
            self.get_logger().info('✅ All commands sent. Shutting down node.')
            # publish zero speeds to stop the robot
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.publisher.publish(msg)
            self.get_logger().info(f'[{now:.2f}s] Published: {msg.data}')
            # stop the timer
            self.destroy_timer(self.timer)

def main(args=None):
    import sys
    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> <this_script> path/to/commands.csv")
        return

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f"❌ File not found: {csv_path}")
        return

    rclpy.init(args=args)
    node = CSVVelocityPlayer(csv_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
