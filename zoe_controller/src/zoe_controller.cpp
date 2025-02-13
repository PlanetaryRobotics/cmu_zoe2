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

#include "zoe_controller/controller.hpp"
#include "zoe_controller/zoe_controller.hpp"

namespace 
{
constexpr auto DEFAULT_COMMAND_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}

namespace zoe_controller {

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

ZoeController::ZoeController() : controller_interface::ControllerInterface() {}

rclcpp::Logger ZoeController::log() { return get_node()->get_logger(); }

controller_interface::CallbackReturn ZoeController::on_init() {
    
    try {
        mParamListener = std::make_shared<ParamListener>(get_node());
        mParams = mParamListener->get_params();

    } catch (std::exception &e) {
        fprintf(stderr, "failed node on_init overriden function");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(log(), "starting the Zoe controller@@@@@@@@@@@@@@@@@");
    // currDriveArc = std::make_shared<zoe2_interfaces::msg::DriveArc>();
    // currDriveArc->radius = 0.0;
    // currDriveArc->speed = 0;
    // currDriveArc->sender = "";
    // currDriveArc->time = 0;
    // currDriveTimer = nullptr;
    // TODO: Change following to load in params from xacro
    controller = std::make_shared<DrivingController>(1.64, 0.325, 1.91, 5.0);

    return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration ZoeController::command_interface_configuration() const {

    std::vector<std::string> conf_names{
        mParams.back_left_wheel + "/velocity",
        mParams.front_left_wheel + "/velocity",
        mParams.back_right_wheel + "/velocity",
        mParams.front_right_wheel + "/velocity",
    };

    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration ZoeController::state_interface_configuration() const {
    std::vector<std::string> conf_names{
        mParams.back_left_wheel + "/velocity",
        mParams.front_left_wheel + "/velocity",
        mParams.back_right_wheel + "/velocity",
        mParams.front_right_wheel + "/velocity",
        mParams.front_yaw + "/position",
        mParams.back_yaw + "/position"
    };

    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

rclcpp::Logger serv_logger() { return rclcpp::get_logger("drive-command"); }

// void ZoeDriveCommandService(
//     ZoeController *impl,
//     const zoe2_interfaces::srv::DriveCommand::Request::SharedPtr request,
//     zoe2_interfaces::srv::DriveCommand::Response::SharedPtr response) {

//     RCLCPP_INFO(impl->log(), "got a command ");

//     impl->currDriveArcMutex.lock();
//     impl->currDriveArc->radius = request->drive_arc.radius;
//     impl->currDriveArc->speed = request->drive_arc.speed;
//     impl->currDriveArc->sender = request->drive_arc.sender;
//     impl->currDriveArc->time = request->drive_arc.time;
//     impl->currDriveArcMutex.unlock();

//     response->success = true;
// }

controller_interface::return_type
ZoeController::update(const rclcpp::Time &time,
                      const rclcpp::Duration &period) {

    RCLCPP_INFO(log(), "Got an update ");

    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);

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
    double & linear_command = command.twist.linear.x;
    double & angular_command = command.twist.angular.z;


    previous_update_timestamp_ = time;

    previous_commands_.pop();
    previous_commands_.emplace(command);

    // Compute wheel velocities

    auto speed = linear_command;
    auto radius = (std::abs(angular_command) < 1e-6) ? std::numeric_limits<double>::max() : (linear_command / angular_command);

    RCLCPP_INFO(log(), "Radius: %f, Speed: %f", radius, speed);


    // currDriveArcMutex.lock();
    // auto speed = currDriveArc->speed;
    // auto radius = currDriveArc->radius;
    // auto sender = currDriveArc->sender;
    // auto time_left = currDriveArc->time;

    controller->setTarget(radius, speed);

    controller->setVfl(frontLeft->feedback_vel.get().get_value() * controller->getWheelRadius());
    controller->setVfr(frontRight->feedback_vel.get().get_value() * controller->getWheelRadius());
    controller->setVbl(backLeft->feedback_vel.get().get_value() * controller->getWheelRadius());
    controller->setVbr(backRight->feedback_vel.get().get_value() * controller->getWheelRadius());

    controller->setThetaf(frontYaw->yaw.get().get_value());
    controller->setThetab(backYaw->yaw.get().get_value());
    
    controller->computeWheelSpeed();
    // currDriveArcMutex.unlock();


    frontLeft->cmd_velocity.get().set_value(controller->getcVfl()/controller->getWheelRadius());
    frontRight->cmd_velocity.get().set_value(controller->getcVfr()/controller->getWheelRadius());
    backLeft->cmd_velocity.get().set_value(controller->getcVbl()/controller->getWheelRadius());
    backRight->cmd_velocity.get().set_value(controller->getcVbr()/controller->getWheelRadius());

    // currDriveArcMutex.lock();
    // currDriveArc->time -= period.seconds();

    // if (currDriveArc->time <= 0) {
    //     currDriveArc->radius = 0;
    //     currDriveArc->speed = 0;
    //     currDriveArc->sender = "";
    //     currDriveArc->time = 0;
    // }
    // currDriveArcMutex.unlock();

    return controller_interface::return_type::OK;
}
// void OnDriveArcMsg(ZoeController *impl,
//                    zoe2_interfaces::msg::DriveArc::ConstSharedPtr driveArc) {
//     RCLCPP_INFO(serv_logger(), "drive arc message %s",
//                 driveArc->sender.c_str());
// }

controller_interface::CallbackReturn
ZoeController::on_configure(const rclcpp_lifecycle::State &) {

    // TODO: setup odom and transform publishers and subscribers

    // drive_command_service =
    //     get_node()->create_service<zoe2_interfaces::srv::DriveCommand>(
    //         "/zoe_drive",
    //         std::bind(&ZoeDriveCommandService, this, std::placeholders::_1,
    //                   std::placeholders::_2));
    // mCommandVelSubscriber =
    //     get_node()->create_subscription<zoe2_interfaces::msg::DriveArc>(
    //         "/drive_cmd_curr", rclcpp::QoS(1),
    //         [this](zoe2_interfaces::msg::DriveArc::ConstSharedPtr arc) {
    //             OnDriveArcMsg(this, arc);
    //         });

    // RCLCPP_INFO(log(), "started /zoe_drive and /drive_cmd_curr");

    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

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
            received_velocity_msg_ptr_.set(std::move(msg));
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
            received_velocity_msg_ptr_.get(twist_stamped);
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = get_node()->get_clock()->now();
            });
    }
    previous_update_timestamp_ = get_node()->get_clock()->now();

    return controller_interface::CallbackReturn::SUCCESS;
}


std::shared_ptr<ZoeController::YawHandle> ZoeController::getYawStateIface(const std::string &name) {
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

std::shared_ptr<ZoeController::WheelHandle>
ZoeController::getWheelHandleByName(const std::string &name) {
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

controller_interface::CallbackReturn
ZoeController::on_activate(const rclcpp_lifecycle::State &state) {
    frontLeft = getWheelHandleByName(mParams.front_left_wheel);
    frontRight = getWheelHandleByName(mParams.front_right_wheel);
    backLeft = getWheelHandleByName(mParams.back_left_wheel);
    backRight = getWheelHandleByName(mParams.back_right_wheel);
    backYaw = getYawStateIface(mParams.back_yaw);
    frontYaw = getYawStateIface(mParams.front_yaw);

    subscriber_is_active_ = true;
    
    RCLCPP_INFO(log(), "SUCCESS: configured WheelHandles and yaw handles");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ZoeController::on_deactivate(const rclcpp_lifecycle::State &) {
    subscriber_is_active_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ZoeController::on_cleanup(const rclcpp_lifecycle::State &) {

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ZoeController::on_error(const rclcpp_lifecycle::State &) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ZoeController::on_shutdown(const rclcpp_lifecycle::State &) {
    return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace zoe_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(zoe_controller::ZoeController,
                            controller_interface::ControllerInterface)
