#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tuple>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include "astar_planner.cpp"
#include "zoe2_interfaces/srv/drive_command.hpp"

class AStarPlannerNode : public rclcpp::Node {
public:
    AStarPlannerNode()
        : Node("a_star_planner_node"),
          robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0),
          axle_radius_(0.325), axle_width_(1.64), axle_wheelbase_(1.91) {
        // Initialize the service client
        // drive_command_client_ = this->create_client<zoe2_interfaces::srv::DriveCommand>("/zoe_drive");
        drive_command_client_ = this->create_client<zoe2_interfaces::srv::DriveCommand>("/zoe_drive");
        // Subscribe to the /odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AStarPlannerNode::odom_callback, this, std::placeholders::_1));

        // Axle configuration assumed to be static for simplicity
        RCLCPP_INFO(this->get_logger(), "A* Planner Node Initialized");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Client<zoe2_interfaces::srv::DriveCommand>::SharedPtr drive_command_client_;

    double robot_x_, robot_y_, robot_theta_;
    double axle_radius_, axle_width_, axle_wheelbase_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Update robot position and orientation from odometry
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_theta_ = atan2(
            2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z),
            1.0 - 2.0 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

        RCLCPP_INFO(this->get_logger(),
                    "Odom Received: x=%.2f, y=%.2f, theta=%.2f",
                    robot_x_, robot_y_, robot_theta_);

        // Trigger A* planning
        plan_path();
    }

    void plan_path() {
        // Goal position (example, can be dynamic)
        double goal_x = 4.0, goal_y = 4.0;

        // A* parameters
        double dt = 0.1;
        double wgt_heur = 5.0;
        double goal_radius = 0.1;
        double th_gain = 0.1;

        // Create the A* planner
        AStarPlanner planner(robot_x_, robot_y_, goal_x, goal_y,
                             {dt, axle_radius_, axle_width_, axle_wheelbase_, wgt_heur, goal_radius, th_gain});

        auto path = planner.a_star();

        int step_number = 1;
        for (const auto& step : path) {
            double radius = std::get<3>(step);
            double speed = std::get<4>(step);
            auto request = std::make_shared<zoe2_interfaces::srv::DriveCommand::Request>();

            request->drive_arc.radius = radius;
            request->drive_arc.speed = speed;
            request->drive_arc.time = step_number * 1000;
            request->drive_arc.sender = "step_" + std::to_string(step_number);

            // Wait for the service to be available
            if (!drive_command_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_ERROR(this->get_logger(), "Service /zoe_drive not available.");
                return;
            }

            auto future = drive_command_client_->async_send_request(request);
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Step %d executed successfully.", step_number);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Step %d execution failed.", step_number);
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }

            ++step_number;
        }
    }
};