#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

class DriveArcVisualizer : public rclcpp::Node {
public:
    DriveArcVisualizer() : Node("drive_arc_visualizer") {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_unstamped", 10, 
            std::bind(&DriveArcVisualizer::cmdVelCallback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg->angular.z == 0.0) {
            RCLCPP_WARN(this->get_logger(), "Zero angular velocity, no arc to visualize.");
            return;
        }

        double radius = msg->linear.x / msg->angular.z;
        double arc_length = 5.0;  // Set a desired arc length in meters
        double theta_max = arc_length / std::abs(radius); // Compute max angle for given arc length
        int num_points = 50;

        // Marker initialization
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "drive_arc";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Angle offset to align arc with the robot's front
        double angle_offset = M_PI / 2;  

        for (int i = 0; i <= num_points; i++) {
            double theta = (i / static_cast<double>(num_points)) * theta_max;

            // Compute the point on the arc
            double x = radius * sin(theta);
            double y = radius * (1 - cos(theta));

            // Rotate the arc points to align with the robot’s heading
            geometry_msgs::msg::Point p;
            p.x = x * cos(angle_offset) - y * sin(angle_offset);
            p.y = x * sin(angle_offset) + y * cos(angle_offset);
            p.z = 0.0;

            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveArcVisualizer>());
    rclcpp::shutdown();
    return 0;
}
