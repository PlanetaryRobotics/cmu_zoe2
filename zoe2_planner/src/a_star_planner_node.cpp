#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tuple>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "zoe2_interfaces/srv/drive_command.hpp"
#include "zoe2_planner/zoe2_astar_planner.h" // Include the new planner definition


class AStarPlannerNode : public rclcpp::Node {
public:
    AStarPlannerNode() : Node("a_star_planner_node"), 
        robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0), goal_x_(4.0), goal_y_(-1.0) {
        
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
    double goal_x_, goal_y_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        double heading = get_heading_from_pose_stamped(msg->pose.pose);
        robot_theta_ = -heading; // future fix: the planner has the angle direction inverted


        RCLCPP_INFO(this->get_logger(),
                    "Odom Received: x=%.2f, y=%.2f, theta=%.2f",
                    robot_x_, robot_y_, robot_theta_);

        // Trigger A* planning
        // check distance to goal
        double dx = goal_x_ - robot_x_;
        double dy = goal_y_ - robot_y_;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < 0.5) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            stop_rover();
            return;
        }
        plan_path();
    }

    void stop_rover() {

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

    void plan_path() {
        auto start_time = std::chrono::high_resolution_clock::now();

        // initialize planner
        double rad = 0.325;
        double width = 1.64;
        double wheelbase = 1.91;
        double wgt_heur = 10;
        double goal_radius = 0.0707;
        double th_gain = 1;
        double steer_angle_smooth = 20;
        std::vector<double> bounds = {-10.0,-10.0,10.0,10.0};

        std::vector<double> poss_R = {2.5, 15, .5};
        std::vector<double> poss_dt = {.5, 6, 0.25};
        std::vector<double> poss_velo = {-1, 1};
        std::vector<std::vector<double>> cost_map(std::ceil(bounds[2])-std::floor(bounds[0]), std::vector<double>(std::ceil(bounds[3])-std::floor(bounds[1]), 0.0));

        // Populate the cost map using the given formula
        // for (int i = 0; i < std::ceil(bounds[2])-std::floor(bounds[0]); ++i) {
        //     for (int j = 0; j < std::ceil(bounds[3])-std::floor(bounds[1]); ++j) {
        //         cost_map[i][j] = std::max(10 - 2 * std::abs(i - j), 0);
        //     }
        // }
        cost_map[0-bounds[1]][-2-bounds[0]] = 10000;

        AStarPlanner planner(
            {rad, width, wheelbase, wgt_heur, goal_radius, th_gain, steer_angle_smooth},
            bounds,
            poss_R,
            poss_dt,
            poss_velo,
            cost_map
        );


        std::vector<std::tuple<double, double, double, double, double, double, double>> path;

        // Compute the paths
        path = planner.a_star(robot_x_, robot_y_, robot_theta_, goal_x_, goal_y_);

        // RCLCPP_INFO(this->get_logger(),
        //             "NEXT STEP COMPUTED");

        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No path found!");
            return;
        }

        // Extract the second step of the path
        auto second_step = path[1];

        double radius = std::get<3>(second_step);
        double dt = std::get<4>(second_step);
        double speed = std::get<5>(second_step);

        auto request = std::make_shared<zoe2_interfaces::srv::DriveCommand::Request>();
        request->drive_arc.radius = radius;
        request->drive_arc.speed = speed;
        request->drive_arc.time = dt; // Time in milliseconds for this step
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

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double>(end_time - start_time).count();
        RCLCPP_INFO(this->get_logger(), "Planning completed in %.4f seconds.", elapsed);
    }

    double get_heading_from_pose_stamped(const geometry_msgs::msg::Pose& pose)
    {
        // Extract the quaternion
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);

        // Convert quaternion to rotation matrix
        tf2::Matrix3x3 m(q);

        // Extract Euler angles from rotation matrix
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // print the yaw to the command line
        RCLCPP_INFO(this->get_logger(), "Yaw: %.2f", yaw);

        // Return yaw (heading) in radians
        return yaw;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
