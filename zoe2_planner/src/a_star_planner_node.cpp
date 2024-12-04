#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tuple>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include "zoe2_interfaces/srv/drive_command.hpp"
#include "astar_planner.cpp" // Include the new planner definition

class AStarPlannerNode : public rclcpp::Node {
public:
    AStarPlannerNode()
        : Node("a_star_planner_node"),
          robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0),
          axle_radius_(0.325), axle_width_(1.64), axle_wheelbase_(1.91),
          goal_x_(5.0), goal_y_(2.0),
          map_bounds_{0, 0, 7, 7},
          poss_R_{-10, 10, 0.5},
          poss_dt_{0.5, 6, 0.25},
          wgt_heur_(10), velocity_(1.0), goal_radius_(0.05), th_gain_(0.1) {
        
        drive_command_client_ = this->create_client<zoe2_interfaces::srv::DriveCommand>("/zoe_drive");

        // Subscribe to the /odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AStarPlannerNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "A* Planner Node Initialized");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Client<zoe2_interfaces::srv::DriveCommand>::SharedPtr drive_command_client_;

    double robot_x_, robot_y_, robot_theta_;
    double axle_radius_, axle_width_, axle_wheelbase_;
    double goal_x_, goal_y_;
    std::vector<double> map_bounds_;
    std::vector<double> poss_R_, poss_dt_;
    double wgt_heur_, velocity_, goal_radius_, th_gain_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {


        // Log raw message values
    RCLCPP_INFO(this->get_logger(),
                "Raw Odom Message: position.x=%.10f, position.y=%.10f, position.z=%.10f",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);


    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    // Extract quaternion components
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // Convert quaternion to yaw (theta)
    robot_theta_ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    RCLCPP_INFO(this->get_logger(),
                "Odom Received: x=%.2f, y=%.2f, theta=%.2f",
                robot_x_, robot_y_, robot_theta_);

        // Trigger A* planning
        plan_path();
    }

    void plan_path() {
    double dt = 2.0; // Time step duration
    auto start_time = std::chrono::high_resolution_clock::now();

    // Initialize planner
    AStarPlanner planner(robot_x_, robot_y_, robot_theta_, goal_x_, goal_y_,
                         {velocity_, axle_radius_, axle_width_, axle_wheelbase_, wgt_heur_, goal_radius_, th_gain_},
                         map_bounds_, poss_R_, poss_dt_);

    // Compute the path
    auto path = planner.a_star();
    RCLCPP_INFO(this->get_logger(),
                "NEXT STEP COMPUTED");

    if (path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No path found!");
        return;
    }

    // Check if the goal has been reached
    if (planner.heuristic(robot_x_, robot_y_) <= goal_radius_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");

        // Send a stop command
        auto stop_request = std::make_shared<zoe2_interfaces::srv::DriveCommand::Request>();
        stop_request->drive_arc.radius = 0.0;
        stop_request->drive_arc.speed = 0.0;
        stop_request->drive_arc.time = 1000; // Time to ensure the stop command is executed
        stop_request->drive_arc.sender = "planner_stop";

        // Log the stop request details
        RCLCPP_INFO(this->get_logger(), "Sending Stop Request: speed=0.0, radius=0.0");

        if (!drive_command_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service /zoe_drive not available for stop command.");
            return;
        }

        auto stop_future = drive_command_client_->async_send_request(stop_request);
        try {
            auto response = stop_future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Stop command executed successfully.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Stop command execution failed.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call for stop command failed: %s", e.what());
        }

        return; // Exit the function since the goal is reached
    }

    // Extract the first step of the path
    auto first_step = path.front();

    double radius = std::get<3>(first_step);
    double speed = std::get<4>(first_step);

    auto request = std::make_shared<zoe2_interfaces::srv::DriveCommand::Request>();
    request->drive_arc.radius = radius;
    request->drive_arc.speed = speed;
    request->drive_arc.time = dt * 1000; // Time in milliseconds for this step
    request->drive_arc.sender = "planner";

    // Log the service call details
    RCLCPP_INFO(this->get_logger(),
                "Sending Service Request: radius=%.2f, speed=%.2f, time=%.2f, sender=%s",
                request->drive_arc.radius, request->drive_arc.speed, request->drive_arc.time / 1000.0,

                request->drive_arc.sender.c_str());

    if (!drive_command_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service /zoe_drive not available.");
        return;
    }

    auto future = drive_command_client_->async_send_request(request);
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Command executed successfully.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Command execution failed.");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double>(end_time - start_time).count();
    RCLCPP_INFO(this->get_logger(), "Planning completed in %.4f seconds.", elapsed);
}

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
