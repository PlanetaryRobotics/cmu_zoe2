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

#ifndef ZOE2_HARDWARE__CAN_HW_HPP_
#define ZOE2_HARDWARE__CAN_HW_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "zoe2_hardware/commander.hpp"
#include "zoe2_hardware/dispatcher.hpp"

namespace zoe2_hardware
{
class Zoe2Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Zoe2Hardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
    
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // CAN pointer
  std::shared_ptr<Command> can_;

  // Dispatcher

  std::shared_ptr<zoe2_hardware::Dispatcher> dispatcher_;

  // Motors
  struct Motor {
    int id;
    std::string joint_name;
    int polarity;
  };

  std::vector<Motor> motors_= 
  {
    {1, "wheel_front_right_joint", -1},
    {2, "wheel_front_left_joint", 1},
    {3, "wheel_back_left_joint", 1},
    {4, "wheel_back_right_joint", -1},
  };

  // Encoders
  struct Encoder {
    int id;
    std::string joint_name;
    double offset;
  };

  std::vector<Encoder> encoders_ = 
  {
    {52, "axle_roll_back_joint", -1.52},
    {51, "axle_yaw_front_joint", 0.12},
    {53, "axle_yaw_back_joint", 0.233},
  };
};

}  // namespace zoe2_hardware

#endif  // ZOE2_HARDWARE__CAN_HW_HPP_
