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

#include "zoe2_hardware/can_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Motor Constants
#define ENCODER_PPR 1024
#define PI 3.14
#define RADTOTICK 650.8194699

// CAN definitions
#define CAN_INTERFACE "can0"
#define CANOPEN_ID_1 1
#define CANOPEN_ID_2 2
#define CANOPEN_ID_3 3
#define CANOPEN_ID_4 4

#define MAX_SPEED 20000

// Define CANOPEN IDs
std::vector<int> motorIDs = {CANOPEN_ID_1, CANOPEN_ID_2, CANOPEN_ID_3, CANOPEN_ID_4};
std::vector<int> encoderIDs = {50, 51, 52};

// Helper Functions
int start_can(std::shared_ptr<Command> can) {
  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Starting CAN Network...");
  
  if (!can->checkOpenResult()) {
      RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Not open");
      return EXIT_FAILURE;
  } 
  
  for (int id : motorIDs) {
      if (can->setOperational(id) < 0) {
          RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Node %i could not be set operational...", id);
          return EXIT_FAILURE;
      }

      if (can->testCan(id) < 0) {
          RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Node %i failed test...", id);
          return EXIT_FAILURE;
      }
  }

  for (int id : encoderIDs) {
    if (can->nmtStart(id) < 0) {
      RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Node %i could not be set operational...", id);
      return EXIT_FAILURE;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "CAN Setup Successful.");
  return EXIT_SUCCESS;
}

int end_can(std::shared_ptr<Command> can){
  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "Ending CAN Network...");
  // run teardown here
  for (int id : motorIDs) {
    can->stop(id);
  }
  //
  RCLCPP_INFO(rclcpp::get_logger("can_hw"), "CAN Teardown Successful.");
  return EXIT_SUCCESS;
}

int rad_to_tick(double rad){
  return int(rad*RADTOTICK);
}

double tick_to_rad(int tick){
  return double(tick)/RADTOTICK;
}

namespace zoe2_hardware
{
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

  // initialize CAN
  can_ = std::make_shared<Command>(CAN_INTERFACE, true);
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
  for (int id:motorIDs){
    can_->configureSpeedMode(id);
  }
  
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
  // TODO(anyone): prepare the robot to receive commands

  RCLCPP_INFO(get_logger(), "Activating... Please wait...");


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
  // TODO(anyone): prepare the robot to stop receiving commands
  RCLCPP_INFO(get_logger(), "Deactivating... Please wait...");


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zoe2Hardware::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/){
  // TODO(anyone): prepare the robot to stop receiving commands
  RCLCPP_INFO(get_logger(), "Shutting down... Please wait...");

  end_can(can_);

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Zoe2Hardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  int iterator = 0;
  int measuredPosition = 0;
  int measuredSpeed = 0;
  std::stringstream ss;
  ss << "Reading states:";
  ss << std::fixed << std::setprecision(2);
  for (const auto & [name, descr] : joint_state_interfaces_){
    if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION){
      if (name.find("wheel") != std::string::npos){
        can_->getPosition(&measuredPosition, motorIDs[iterator++]);
        set_state(name, tick_to_rad(measuredPosition));
        // ss << std::endl
        //   << "\t position " << get_state(name) << " and velocity " << velo << " for '" << name
        //   << "'!";
      }
      else if (descr.get_interface_name() == hardware_interface::HW_IF_VELOCITY){
        can_->getSpeed(&measuredSpeed, motorIDs[iterator++]);
        set_state(name, tick_to_rad(measuredSpeed));
      }
    }
  }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zoe2Hardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  std::stringstream ss;
  ss << "Writing commands:";

  int iterator = 0;

  for (const auto & [name, descr] : joint_command_interfaces_){
    // Simulate sending commands to the hardware
    set_state(name, get_command(name));

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << "command " << get_command(name) << " for '" << name << "'!";

    int speed_ticks = int(rad_to_tick(get_command(name)));
    ss << std::fixed << std::setprecision(2) << std::endl
      << "\t" << "speed ticks " << speed_ticks << " for '" << name << "'!";

    // cap the magnitude, but respect the sign
    speed_ticks = std::min(std::max(speed_ticks, -MAX_SPEED), MAX_SPEED);

    can_->setSpeed(speed_ticks, motorIDs[iterator++]);

  }

  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());  

  return hardware_interface::return_type::OK;
}

}  // namespace zoe2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  zoe2_hardware::Zoe2Hardware, hardware_interface::SystemInterface)
