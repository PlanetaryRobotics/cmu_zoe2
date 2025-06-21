#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_msgs/msg/DynamicJointState.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <algorithm>

// State of the robot
struct State {
  double x_world;
  double y_world;
  double phi_world;
  double theta_front_axle;
  double theta_rear_axle;

  State() {}
  State(double x_, double y_, double phi_, double theta_f_, double theta_r_) : x_world(x_), y_world(y_), phi_world(phi_), theta_front_axle(theta_f_), theta_rear_axle(theta_r_) {}
};

// Input of the mobile robot
struct Input {
  double omega_fl;
  double omega_fr;
  double omega_rl;
  double omega_rr;

  Input() {}
  Input(double omega_fl_, double omega_fr_, double omega_rl_, double omega_rr_) : omega_fl(omega_fl_), omega_fr(omega_fr_), omega_rl(omega_rl_), omega_rr(omega_rr_) {}
};

class OptimalController : public rclcpp::Node
{
  public:
    OptimalController() : Node("optimal_controller") {
      robot_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OptimalController::robotOdomCallback, this, std::placeholders::_1)); // Need to change function for callback
      
      robot_joint_sub_ = this->create_subscription<control_msgs/msg/DynamicJointState>(
        "/dynamic_joint_states", 10, std::bind(&OptimalController::robotJointCallback, this, std::placeholders::_1)); // Need to change function for callback
      
      control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_unstamped", 10);
      
    }
  
  private:


    // Subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr robot_joint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_input_pub_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world zoe2_optimal_controller package\n");
  return 0;
}
