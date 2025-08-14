// Copyright (c) 2025, cmu
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>

#include "zoe2_hardware/can_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Motor Constants
#define ENCODER_PPR 1024
#define PI 3.14159265359

// CAN definitions
#define CAN_INTERFACE "can0"

#define MAX_SPEED 100000

#define GEARING 50  // gear ratio of harmonic drive

namespace zoe2_hardware
{

// {CANOPEN ID, Joint mapping, polarity}
std::vector<Motor> motors_ = 
  {
    {1, "wheel_front_right_joint", -1},
    {2, "wheel_front_left_joint", 1},
    {3, "wheel_back_left_joint", 1},
    {4, "wheel_back_right_joint", -1},
  };

// {CANOPEN ID, Joint mapping, angle offset}
std::vector<Encoder> encoders_ = 
  {
    {50, "axle_roll_back_joint", 0.0},
    {52, "axle_yaw_front_joint", 0.0},
    {53, "axle_yaw_back_joint", 0.0},
  };

// Helper Functions
int start_can(std::shared_ptr<Command> can) {
  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Starting CAN Network...");
  
  if (!can->checkOpenResult()) {
      RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Not open");
      return EXIT_FAILURE;
  } 
  
  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "CAN Setup Successful.");
  return EXIT_SUCCESS;
}

int end_can(std::shared_ptr<Command> can){
  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Ending CAN Network...");
  // run teardown here
  for (const auto& [id, name] : motors) {
    can->stop(id);
  }
  //
  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "CAN Teardown Successful.");
  return EXIT_SUCCESS;
}

constexpr double TICK_PER_RAD = 4096.0 / (2.0 * PI);

int rad_to_tick(double rad) {
    return static_cast<int>(rad * TICK_PER_RAD);
}

double tick_to_rad(int tick) {
    return tick / TICK_PER_RAD;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_init(const hardware_interface::HardwareInfo & info){
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Initializing ...please wait...");

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  RCLCPP_INFO(get_logger(), "Configuring... please wait...");

  // creating Can instance 
  can_ = std::make_shared<Command>(CAN_INTERFACE, true);


  // creating Dispatcher Before CAN init
  int socket = can_->getSocketFD();
  dispatcher_ = std::make_shared<zoe2_hardware::Dispatcher>(socket); // have dispatcher accessable to can.cpp
  RCLCPP_INFO(get_logger(), "Dispatcher Created!");

  can_->setDispatcher(dispatcher_);
  dispatcher_->start();

  // initialize CAN
  start_can(can_);

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    // TODO: Read states
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  // set each can to velocity mode
  for (const Motor& motor : motors_){
    can_->configureSpeedMode(motor.id);
  }
  



  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
  // TODO(anyone): prepare the robot to receive commands

  RCLCPP_INFO(get_logger(), "Activating... Please wait...");

  for (const Motor& motor : motors_) {
      if (can_->setOperational(motor.id) < 0) {
          RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Node %i could not be set operational...", motor.id);
          return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Testing CAN ID: %i", motor.id);
      if (can_->testCan(motor.id) < 0) {

          RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Node %i failed test...", motor.id);
          return CallbackReturn::ERROR;
      }
  }

  for (const Encoder & encoder : encoders_) {
    if (can_->nmtStart(encoder.id) < 0) {
      RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Node %i could not be set operational...", encoder.id);
      return CallbackReturn::ERROR;
    }
  }


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
  // TODO(anyone): prepare the robot to stop receiving commands
  RCLCPP_INFO(get_logger(), "Deactivating... Please wait...");

  for (const auto& encoder : encoders_) {
    if (can_->nmtStop(encoder.id) < 0) {
      RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Node %i could not be stopped...", encoder.id);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
  // TODO(anyone): prepare the robot to stop receiving commands
  RCLCPP_INFO(get_logger(), "Shutting down... Please wait...");

  end_can(can_);

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Zoe2Hardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  
  // READ MOTOR VALUES FROM DISPATCHER
  for (const auto& motor : motors_) {
    int measuredPosition = 0;
    int measuredSpeed = 0;

    // Get Position
    can_->getPosition(&measuredPosition, motor.id);
    set_state(motor.joint_name + "/position", tick_to_rad(measuredPosition*motor.polarity));
    // Get Speed
    can_->getSpeed(&measuredSpeed, motor.id);
    set_state(motor.joint_name + "/velocity", tick_to_rad(measuredSpeed*motor.polarity));
  }

  // READ ENCODER VALUES FROM DISPATCHER
  struct can_frame temp_frame;

  for (const auto& encoder : encoders_) {
    temp_frame = (dispatcher_->getMessagesForId(encoder.id)).front();
    uint32_t position = (temp_frame.data[3] <<24)|(temp_frame.data[2] <<16)|(temp_frame.data[1] <<8)|(temp_frame.data[0]);
    double data = std::fmod((tick_to_rad(position)-encoder.offset),2*PI) - PI;
    set_state(encoder.joint_name + "/position", data);
  }


  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zoe2Hardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  std::stringstream ss;
  ss << "Writing commands:";

  for (const auto & motor : motors_) {
    std::string joint = motor.joint_name + "/velocity";

    // Simulate sending commands to the hardware
    set_state(joint, get_command(joint));

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << "command " << get_command(joint) << " for '" << joint << "'!";

    int speed_ticks = int(rad_to_tick(get_command(joint))*GEARING*motor.polarity);
    ss << std::fixed << std::setprecision(2) << std::endl
      << "\t" << "speed ticks " << speed_ticks << " for '" << joint << "'!";

    // cap the magnitude, but respect the sign
    speed_ticks = std::min(std::max(speed_ticks, -MAX_SPEED), MAX_SPEED);

    can_->setSpeed(speed_ticks, motor.id);

  }

  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());  

  return hardware_interface::return_type::OK;
}

}  // namespace zoe2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  zoe2_hardware::Zoe2Hardware, hardware_interface::SystemInterface)
