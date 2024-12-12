#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tuple>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include "zoe2_interfaces/srv/generate_drive_plan.hpp"
#include "zoe2_interfaces/msg/drive_plan.hpp"
#include "zoe2_interfaces/msg/drive_arc.hpp"
#include "zoe2_planner/zoe2_astar_planner.h" // Include the new planner definition

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double get_heading_from_pose_stamped(const geometry_msgs::msg::PoseStamped& pose_stamped);

void plan_path(const std::shared_ptr<zoe2_interfaces::srv::GenerateDrivePlan::Request> request,
          std::shared_ptr<zoe2_interfaces::srv::GenerateDrivePlan::Response>      response)
{
    // the request is coming in with the following info:
    // geometry_msgs/PoseStamped current_pose
    // geometry_msgs/PoseStamped goal_pose

    // we want to extract the start and goal position from that.

    // current pose has orientation as a quaternion. We want to convert to radians and get the angle
    // use the tf2 library to convert the quaternion to an angle

    double heading = get_heading_from_pose_stamped(request->current_pose);

    std::vector<double> init = {request->current_pose.pose.position.x, 
                                request->current_pose.pose.position.y, 
                                heading};
    std::vector<double> goal = {request->goal_pose.pose.position.x,
                                request->goal_pose.pose.position.y};

	// print out the start and goal positions
	std::cout << "Received Start: " << init[0] << " " << init[1] << " " << init[2] << std::endl;
	std::cout << "Received Goal: " << goal[0] << " " << goal[1] << std::endl;


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
    std::vector<double> poss_velo = {-1,1};

    std::vector<std::vector<double>> cost_map(std::ceil(bounds[2])-std::floor(bounds[0]), std::vector<double>(std::ceil(bounds[3])-std::floor(bounds[1]), 0.0));

    // Populate the cost map using the given formula
    // for (int i = 0; i < std::ceil(bounds[2])-std::floor(bounds[0]); ++i) {
    //     for (int j = 0; j < std::ceil(bounds[3])-std::floor(bounds[1]); ++j) {
    //         cost_map[i][j] = std::max(10 - 2 * std::abs(i - j), 0);
    //     }
    // }
    cost_map[0-bounds[1]][-2-bounds[0]] = 10000;

    // Create the AStarPlanner object using constructor parameters
    AStarPlanner planner(
        {rad, width, wheelbase, wgt_heur, goal_radius, th_gain, steer_angle_smooth},
        bounds,
        poss_R,
        poss_dt,
        poss_velo,
        cost_map
    );

    std::vector<std::tuple<double, double, double, double, double, double, double>> path;

    path = planner.a_star(init[0], init[1], init[2], goal[0], goal[1]);

    // path output is formatted as follows:
    // curr.x, curr.y, curr.theta, curr.arc_radius, curr.dt, curr.v, curr.g
    
    // our path message is formatted as an array of arcs, which each are as follows:
    // arc_radius, speed, time, sender

    // write the path message: response
    for (int i = 0; i < path.size(); i++) {
        std::tuple<double, double, double, double, double, double, double> curr = path[i];
        // Create a DriveArc message
        zoe2_interfaces::msg::DriveArc arc;
        arc.radius = std::get<3>(curr); // Replace 'curr' with your source of data
        arc.speed = std::get<5>(curr);
        arc.time = std::get<4>(curr);
        arc.sender = "planner";
        response->drive_plan.drive_arcs.push_back(arc);
    }

    // Print the path to the console
	std::cout << "Generated path: " << std::endl;
    for (int i = 0; i < path.size(); i++) {
        std::tuple<double, double, double, double, double, double, double> curr = path[i];
        std::cout << "x: " << std::get<0>(curr) << " y: " << std::get<1>(curr) << " theta: " << std::get<2>(curr) << " arc_radius: " << std::get<3>(curr) << " dt: " << std::get<4>(curr) << " v: " << std::get<5>(curr) << " g: " << std::get<6>(curr) << std::endl;
    }
	
    
}


double get_heading_from_pose_stamped(const geometry_msgs::msg::PoseStamped& pose_stamped)
{
    // Extract the quaternion
    tf2::Quaternion q(
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w);

    // Convert quaternion to rotation matrix
    tf2::Matrix3x3 m(q);

    // Extract Euler angles from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Return yaw (heading) in radians
    return yaw;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("astar_server");

  rclcpp::Service<zoe2_interfaces::srv::GenerateDrivePlan>::SharedPtr service =
    node->create_service<zoe2_interfaces::srv::GenerateDrivePlan>("plan_path", &plan_path);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to plan path.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}