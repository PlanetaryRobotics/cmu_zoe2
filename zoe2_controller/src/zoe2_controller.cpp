// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "zoe2_controller/controller.hpp"
#include "zoe2_controller/zoe2_controller.hpp"

namespace 
{
constexpr auto DEFAULT_COMMAND_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}

namespace zoe2_controller {

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

Zoe2Controller::Zoe2Controller() : controller_interface::ControllerInterface() {}

rclcpp::Logger Zoe2Controller::log() { return get_node()->get_logger(); }

controller_interface::CallbackReturn Zoe2Controller::on_init() {
    
    try {
        mParamListener = std::make_shared<ParamListener>(get_node());
        mParams = mParamListener->get_params();

    } catch (std::exception &e) {
        fprintf(stderr, "failed node on_init overriden function");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(log(), "starting the Zoe controller@@@@@@@@@@@@@@@@@");
    controller = std::make_shared<DrivingController>(mParams.base_width, mParams.wheel_radius, mParams.robot_length, mParams.proportional_gain);

    return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration Zoe2Controller::command_interface_configuration() const {

    std::vector<std::string> conf_names{
        mParams.wheel_back_left + "/velocity",
        mParams.wheel_front_left + "/velocity",
        mParams.wheel_back_right + "/velocity",
        mParams.wheel_front_right + "/velocity",
    };

    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration Zoe2Controller::state_interface_configuration() const {
    std::vector<std::string> conf_names{
        mParams.wheel_back_left + "/velocity",
        mParams.wheel_front_left + "/velocity",
        mParams.wheel_back_right + "/velocity",
        mParams.wheel_front_right + "/velocity",
        mParams.axle_yaw_front + "/position",
        mParams.axle_yaw_back + "/position"
    };

    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type
Zoe2Controller::update(const rclcpp::Time &time,
                      const rclcpp::Duration & /*period*/) {

    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get([&](const auto &msg) {
        last_command_msg = msg;
    });

    if (last_command_msg == nullptr)
    {
        RCLCPP_WARN(log(), "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
        last_command_msg->twist.linear.x = 0.0;
        last_command_msg->twist.angular.z = 0.0;
    }

    // command may be limited further by SpeedLimit,
    // without affecting the stored twist command
    Twist command = *last_command_msg;
    previous_update_timestamp_ = time;

    previous_commands_.pop();
    previous_commands_.emplace(command);

    // Compute wheel velocities

    controller->setDriveCommand(command.twist.linear.x, command.twist.angular.z);

    controller->setVfl(frontLeft->feedback_vel.get().get_optional().value() * controller->getWheelRadius());
    controller->setVfr(frontRight->feedback_vel.get().get_optional().value() * controller->getWheelRadius());
    controller->setVbl(backLeft->feedback_vel.get().get_optional().value() * controller->getWheelRadius());
    controller->setVbr(backRight->feedback_vel.get().get_optional().value() * controller->getWheelRadius());

    controller->setThetaf(frontYaw->yaw.get().get_optional().value());
    controller->setThetab(backYaw->yaw.get().get_optional().value());
    
    controller->computeWheelSpeed();

    setVelocityCommand(frontLeft->cmd_velocity.get(), controller->getcVfl() / controller->getWheelRadius(), "front left", log());
    setVelocityCommand(frontRight->cmd_velocity.get(), controller->getcVfr() / controller->getWheelRadius(), "front right", log());
    setVelocityCommand(backLeft->cmd_velocity.get(), controller->getcVbl() / controller->getWheelRadius(), "back left", log());
    setVelocityCommand(backRight->cmd_velocity.get(), controller->getcVbr() / controller->getWheelRadius(), "back right", log());
    
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
Zoe2Controller::on_configure(const rclcpp_lifecycle::State &) {

    const Twist empty_twist;
    received_velocity_msg_ptr_.set([&](auto &msg) {
        msg = std::make_shared<Twist>(empty_twist);
    });

    // Fill last two commands with default constructed commands
    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    // initialize command subscriber
    if (use_stamped_vel_)
    {
        velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<Twist> msg) -> void
        {
            if (!subscriber_is_active_)
            {
            RCLCPP_WARN(
                get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
            {
            RCLCPP_WARN_ONCE(
                get_node()->get_logger(),
                "Received TwistStamped with zero timestamp, setting it to current "
                "time, this message will only be shown once");
            msg->header.stamp = get_node()->get_clock()->now();
            }
            received_velocity_msg_ptr_.set([&](auto& value) { value = std::move(msg); });
        });
    }
    else
    {
        velocity_command_unstamped_subscriber_ =
        get_node()->create_subscription<geometry_msgs::msg::Twist>(
            DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
            {
            if (!subscriber_is_active_)
            {
                RCLCPP_WARN(
                get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                return;
            }

            // Write fake header in the stored stamped command
            std::shared_ptr<Twist> twist_stamped;
            received_velocity_msg_ptr_.get([&](const auto& value) { twist_stamped = value; });
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = get_node()->get_clock()->now();
            });
    }
    previous_update_timestamp_ = get_node()->get_clock()->now();

    return controller_interface::CallbackReturn::SUCCESS;
}


std::shared_ptr<Zoe2Controller::YawHandle> Zoe2Controller::getYawStateIface(const std::string &name) {
    auto state_iface = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&name](const hardware_interface::LoanedStateInterface &interface) {
            return interface.get_prefix_name() == name &&
                   interface.get_interface_name() == HW_IF_POSITION;
        });

    if (state_iface == state_interfaces_.end()) {
        RCLCPP_INFO(log(), "invalid state name %s", name.c_str());
        return nullptr;
    }
    return std::make_shared<YawHandle>(std::ref(*state_iface));
}

std::shared_ptr<Zoe2Controller::WheelHandle>
Zoe2Controller::getWheelHandleByName(const std::string &name) {
    auto state_iface = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&name](const hardware_interface::LoanedStateInterface &interface) {
            return interface.get_prefix_name() == name &&
                   interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (state_iface == state_interfaces_.end()) {
        RCLCPP_INFO(log(), "invalid state name %s", name.c_str());
        return nullptr;
    }
    auto cmd_iface = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&name](const hardware_interface::LoanedCommandInterface &interface) {
            return interface.get_prefix_name() == name &&
                   interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (cmd_iface == command_interfaces_.end()) {

        RCLCPP_INFO(log(), "invalid command name %s", name.c_str());
        return nullptr;
    }
    auto shared = std::make_shared<WheelHandle>(std::ref(*state_iface),
                                                std::ref(*cmd_iface));
    return shared;
}

void Zoe2Controller::setVelocityCommand(
    hardware_interface::LoanedCommandInterface &cmd_interface, 
    double value, 
    const std::string &wheel_name, 
    rclcpp::Logger logger) 
{
    if (!cmd_interface.set_value(value)) {
        RCLCPP_ERROR(logger, "Failed to set velocity for %s", wheel_name.c_str());
    }
}

controller_interface::CallbackReturn
Zoe2Controller::on_activate(const rclcpp_lifecycle::State & /*state*/) {
    frontLeft = getWheelHandleByName(mParams.wheel_front_left);
    frontRight = getWheelHandleByName(mParams.wheel_front_right);
    backLeft = getWheelHandleByName(mParams.wheel_back_left);
    backRight = getWheelHandleByName(mParams.wheel_back_right);
    backYaw = getYawStateIface(mParams.axle_yaw_back);
    frontYaw = getYawStateIface(mParams.axle_yaw_front);

    subscriber_is_active_ = true;
    
    RCLCPP_INFO(log(), "SUCCESS: configured WheelHandles and yaw handles");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Zoe2Controller::on_deactivate(const rclcpp_lifecycle::State &) {
    subscriber_is_active_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Zoe2Controller::on_cleanup(const rclcpp_lifecycle::State &) {

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Zoe2Controller::on_error(const rclcpp_lifecycle::State &) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Zoe2Controller::on_shutdown(const rclcpp_lifecycle::State &) {
    return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace zoe2_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(zoe2_controller::Zoe2Controller,
                            controller_interface::ControllerInterface)
