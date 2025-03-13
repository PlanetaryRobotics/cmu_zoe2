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
        double radius;
        if (msg->angular.z < 0.01) {
            // set a large radius to avoid division by zero
            radius = 9999999.0;
        }
        else {
            radius = - msg->linear.x / msg->angular.z;
        }

        {
        // marker for body velocity
            visualization_msgs::msg::Marker vel_arrow;
            vel_arrow.header.frame_id = "base_link";
            vel_arrow.header.stamp = this->now();
            vel_arrow.ns = "velocity_arrow";
            vel_arrow.id = 0;
            vel_arrow.type = visualization_msgs::msg::Marker::ARROW;
            vel_arrow.action = visualization_msgs::msg::Marker::ADD;
            vel_arrow.scale.x = 0.05;   // Shaft width
            vel_arrow.scale.y = 0.1;    // arrowhead width
            vel_arrow.color.a = 1.0;
            vel_arrow.color.r = 0.0;
            vel_arrow.color.g = 1.0;
            vel_arrow.color.b = 1.0;

            // start point of arrow
            geometry_msgs::msg::Point start;
            start.x = 0.0;
            start.y = 0.0;
            start.z = 1.0;
            vel_arrow.points.push_back(start);

            // end point of arrow
            geometry_msgs::msg::Point end;
            end.x = 0.0;
            end.y = msg->linear.x;
            end.z = 1.0;
            vel_arrow.points.push_back(end);

            // Publish the ICR marker
            marker_pub_->publish(vel_arrow);
        }

        {
            // Marker initialization for the Instantaneous Center of Rotation (ICR) dotted line
            visualization_msgs::msg::Marker icr_line;
            icr_line.header.frame_id = "base_link";
            icr_line.header.stamp = this->now();
            icr_line.ns = "drive_arc_icr_line";
            icr_line.id = 1;
            icr_line.type = visualization_msgs::msg::Marker::POINTS;
            icr_line.action = visualization_msgs::msg::Marker::ADD;
            icr_line.scale.x = 0.1;  // Point width
            icr_line.scale.y = 0.1;  // Point height
            icr_line.color.a = 1.0;
            icr_line.color.r = 0.0;
            icr_line.color.g = 1.0;
            icr_line.color.b = 1.0;

            // Add points to create a dotted line
            int num_points = 10;
            for (int i = 0; i <= num_points; ++i) {
                geometry_msgs::msg::Point p;
                p.x = radius * (i / static_cast<double>(num_points));
                p.y = 0.0;
                p.z = 1.0;
                icr_line.points.push_back(p);
            }

            // Publish the ICR dotted line marker
            marker_pub_->publish(icr_line);
        }
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
