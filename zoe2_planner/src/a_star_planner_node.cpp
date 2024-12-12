#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tuple>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "zoe2_interfaces/srv/drive_command.hpp"
#include "zoe2_interfaces/srv/generate_drive_plan.hpp"

class AStarPlannerNode : public rclcpp::Node {
public:
    AStarPlannerNode()
        : Node("a_star_planner_node") {
        
        path_plan_client_ = this->create_client<zoe2_interfaces::srv::GenerateDrivePlan>("/plan_path");

        // drive_command_client_ = this->create_client<zoe2_interfaces::srv::DriveCommand>("/zoe_drive");

        // Subscribe to the /odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AStarPlannerNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "A* Planner Node Initialized");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    // rclcpp::Client<zoe2_interfaces::srv::DriveCommand>::SharedPtr drive_command_client_;
    rclcpp::Client<zoe2_interfaces::srv::GenerateDrivePlan>::SharedPtr path_plan_client_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Log raw message values
        RCLCPP_INFO(this->get_logger(),
                    "Raw Odom Message: position.x=%.10f, position.y=%.10f, position.z=%.10f",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);


        // we need to convert this to type geometry_msgs/PoseStamped
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;

        call_plan_path();
    }

    void call_plan_path() {
        // Wait for the service to be available
        while (!path_plan_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<zoe2_interfaces::srv::GenerateDrivePlan::Request>();
        request->current_pose = current_pose_;
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.pose.position.x = 5.0;
        goal_pose.pose.position.y = 5.0;
        request->goal_pose = goal_pose;

        auto result = path_plan_client_->async_send_request(request);

        // handle the response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Path Planning Service Call Successful");
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "Got a path");

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
