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
// #include "zoe2_interfaces/msg/drive_arc.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "zoe_controller/controller.hpp"
#include "zoe_controller/zoe_controller.hpp"
// #include "zoe_motor_hardware/commander.hpp"

#define CAN_INTERFACE "can0"
#define CANOPEN_ID_1 1
#define CANOPEN_ID_2 2
#define CANOPEN_ID_3 3
#define CANOPEN_ID_4 4
#define ENCODER_PPR 1024

#define PI 3.14

namespace zoe_controller {

constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

ZoeController::ZoeController() : controller_interface::ControllerInterface() {}

rclcpp::Logger ZoeController::log() { return get_node()->get_logger(); }

controller_interface::CallbackReturn ZoeController::on_init() {

    if(test_hardware) {
        // connection = std::make_shared<Command>(CAN_INTERFACE, true);
        // if(!connection->checkOpenResult()) {
        //     return controller_interface::CallbackReturn::ERROR;
        // } 
        // if(connection->setOperational(CANOPEN_ID_1) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // } 
        // if(connection->setOperational(CANOPEN_ID_2) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // }
        // if(connection->setOperational(CANOPEN_ID_3) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // } 
        // if(connection->setOperational(CANOPEN_ID_4) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // }

        // if (connection->testCan(CANOPEN_ID_1) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // } 
        // if (connection->testCan(CANOPEN_ID_2) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // } 
        // if (connection->testCan(CANOPEN_ID_3) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // } 
        // if (connection->testCan(CANOPEN_ID_4) < 0) {
        //     return controller_interface::CallbackReturn::ERROR;
        // }
    }
    

    try {
        mParamListener = std::make_shared<ParamListener>(get_node());
        mParams = mParamListener->get_params();

    } catch (std::exception &e) {
        fprintf(stderr, "failed node on_init overriden function");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(log(), "starting the Zoe controller@@@@@@@@@@@@@@@@@");
    currDriveArc = std::make_shared<zoe2_interfaces::msg::DriveArc>();
    currDriveArc->radius = 0.0;
    currDriveArc->speed = 0;
    currDriveArc->sender = "";
    currDriveArc->time = 0;
    currDriveTimer = nullptr;
    // TODO: Change following to load in params from xacro
    controller = std::make_shared<DrivingController>(1.64, 0.325, 1.91, 2.0);

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

void ZoeDriveCommandService(
    ZoeController *impl,
    const zoe2_interfaces::srv::DriveCommand::Request::SharedPtr request,
    zoe2_interfaces::srv::DriveCommand::Response::SharedPtr response) {

    RCLCPP_INFO(impl->log(), "got a command ");

    impl->currDriveArcMutex.lock();
    impl->currDriveArc->radius = request->drive_arc.radius;
    impl->currDriveArc->speed = request->drive_arc.speed;
    impl->currDriveArc->sender = request->drive_arc.sender;
    impl->currDriveArc->time = request->drive_arc.time;
    impl->currDriveArcMutex.unlock();

    response->success = true;
}

controller_interface::return_type
ZoeController::update(const rclcpp::Time &time,
                      const rclcpp::Duration &period) {

    currDriveArcMutex.lock();
    auto speed = currDriveArc->speed;
    auto radius = currDriveArc->radius;
    auto sender = currDriveArc->sender;
    auto time_left = currDriveArc->time;

    controller->setTarget(radius, speed);

    if(test_hardware) {
        // double speed1 = 0; double speed2 = 0; double speed3 = 0; double  speed4 = 0;
        // int count1 = 0; int count2 = 0; int count3 = 0; int  count4 = 0;

        
        // connection->getSpeed(&count1, CANOPEN_ID_1);
        // connection->getSpeed(&count2, CANOPEN_ID_2);
        // connection->getSpeed(&count3, CANOPEN_ID_3);
        // connection->getSpeed(&count4, CANOPEN_ID_4);

        // std::cout << "READ COUNTS ARE " << count1 << " " << count2 << " " << count3 << " " << count4 << std::endl;
        // speed1 = connection->get_speed_counts(count1);
        // speed2 = connection->get_speed_counts(count2);
        // speed3 = connection->get_speed_counts(count3);
        // speed4 = connection->get_speed_counts(count4);

        // controller->setVfl(speed1 * controller->getWheelRadius());
        // controller->setVfr(speed1 * controller->getWheelRadius());
        // controller->setVbl(speed1 * controller->getWheelRadius());
        // controller->setVbr(speed1 * controller->getWheelRadius());

        // //TODO: Need to modify this for the hardware test from the digital encoder/potentiometer
        // controller->setThetaf(frontYaw->yaw.get().get_value());
        // controller->setThetab(backYaw->yaw.get().get_value());
    } else {
        controller->setVfl(frontLeft->feedback_vel.get().get_value() * controller->getWheelRadius());
        controller->setVfr(frontRight->feedback_vel.get().get_value() * controller->getWheelRadius());
        controller->setVbl(backLeft->feedback_vel.get().get_value() * controller->getWheelRadius());
        controller->setVbr(backRight->feedback_vel.get().get_value() * controller->getWheelRadius());

        controller->setThetaf(frontYaw->yaw.get().get_value());
        controller->setThetab(backYaw->yaw.get().get_value());
    }
    
    controller->computeWheelSpeed();
    // RCLCPP_INFO(log(), "curr speed: %f radius: %f sender:%s time_left: %f",
    //             speed, radius, sender.c_str(), time_left);
    // RCLCPP_INFO(log(),
    //             "curr speed: \n\t front: \n\t\t left:%f \n\t\t right: %f \n\t "
    //             "back: \n\t\t left: %f \n\t\t right: %f",
    //             controller->getVfl(), controller->getVfr(),
    //             controller->getVbl(), controller->getVbr());
    // RCLCPP_INFO(log(),  "frontRoll:  %f backRoll: %f", frontRoll->yaw.get().get_value(), backRoll->yaw.get().get_value());
    RCLCPP_INFO(log(),
            "curr command speed: \n\t front: \n\t\t left:%f \n\t\t right: %f \n\t "
            "back: \n\t\t left: %f \n\t\t right: %f",
            controller->getcVfl(), controller->getcVfr(),
            controller->getcVbl(), controller->getcVbr());
    currDriveArcMutex.unlock();

    if(test_hardware) {
        
        // int speed1 = 0; int speed2 = 0; int speed3 = 0; int  speed4 = 0;

        // speed1 = connection->get_counts(controller->getcVfl()/controller->getWheelRadius());
        // speed2 = connection->get_counts(controller->getcVfr()/controller->getWheelRadius());
        // speed3 = connection->get_counts(controller->getcVbl()/controller->getWheelRadius());
        // speed4 = connection->get_counts(controller->getcVbr()/controller->getWheelRadius());

        // std::cout << "WRITE SPEEDS ARE " << speed1 << " " << speed2 <<  " " << speed3 << " " << speed4 << std::endl;

        // connection->setSpeed(speed1, CANOPEN_ID_1);
        // connection->setSpeed(speed2, CANOPEN_ID_2);
        // connection->setSpeed(speed3, CANOPEN_ID_3);
        // connection->setSpeed(speed4, CANOPEN_ID_4);
    } else {
        frontLeft->cmd_velocity.get().set_value(controller->getcVfl()/controller->getWheelRadius());
        frontRight->cmd_velocity.get().set_value(controller->getcVfr()/controller->getWheelRadius());
        backLeft->cmd_velocity.get().set_value(controller->getcVbl()/controller->getWheelRadius());
        backRight->cmd_velocity.get().set_value(controller->getcVbr()/controller->getWheelRadius());
    }



    currDriveArcMutex.lock();
    currDriveArc->time -= period.seconds();

    if (currDriveArc->time <= 0) {
        currDriveArc->radius = 0;
        currDriveArc->speed = 0;
        currDriveArc->sender = "";
        currDriveArc->time = 0;
    }
    currDriveArcMutex.unlock();

    return controller_interface::return_type::OK;
}
void OnDriveArcMsg(ZoeController *impl,
                   zoe2_interfaces::msg::DriveArc::ConstSharedPtr driveArc) {
    RCLCPP_INFO(serv_logger(), "drive arc message %s",
                driveArc->sender.c_str());
}

controller_interface::CallbackReturn
ZoeController::on_configure(const rclcpp_lifecycle::State &) {

    // TODO: setup odom and transform publishers and subscribers

    drive_command_service =
        get_node()->create_service<zoe2_interfaces::srv::DriveCommand>(
            "/zoe_drive",
            std::bind(&ZoeDriveCommandService, this, std::placeholders::_1,
                      std::placeholders::_2));
    mCommandVelSubscriber =
        get_node()->create_subscription<zoe2_interfaces::msg::DriveArc>(
            "/drive_cmd_curr", rclcpp::QoS(1),
            [this](zoe2_interfaces::msg::DriveArc::ConstSharedPtr arc) {
                OnDriveArcMsg(this, arc);
            });

    RCLCPP_INFO(log(), "started /zoe_drive and /drive_cmd_curr");

    // mCommandVelSubscriber =
    // get_node()->create_subscription<zoe2_interfaces::msg::DriveArc>(
    //     "cmd_vel", rclcpp::QoS(1),
    //     std::bind(&ZoeController::OnDriveArcMsg, this,
    //     std::placeholders::_1));
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
    
    RCLCPP_INFO(log(), "SUCCESS: configured WheelHandles and yaw handles");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ZoeController::on_deactivate(const rclcpp_lifecycle::State &) {
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
