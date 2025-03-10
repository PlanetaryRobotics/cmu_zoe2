// Copyright 2021 ros2_control Development Team
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

#include "zoe_motor_hardware/zoe_motor_hardware.hpp"


#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zoe_motor_hardware/commander.hpp"

#define CAN_INTERFACE "can0"
#define CANOPEN_ID_1 1
#define CANOPEN_ID_2 2
#define CANOPEN_ID_3 3
#define CANOPEN_ID_4 4


namespace zoe_motor_hardware {

rclcpp::Logger log() { return rclcpp::get_logger("ZoeMotorHardware"); }

hardware_interface::CallbackReturn
ZoeMotorHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    backLeftCmdVel, backRightCmdVel, frontLeftCmdVel, frontRightCmdVel = 0;

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // DiffBotSystem has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // TODO: check what needs to be initialized
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::ComponentInfo>::iterator
ZoeMotorHardware::getJointComponentByName(const std::string &name) {
    auto res =
        std::find_if(info_.joints.begin(), info_.joints.end(),
                     [&name](hardware_interface::ComponentInfo &componentInfo) {
                         return componentInfo.name == name;
                     });

    if (res == info_.joints.end()) {
        RCLCPP_ERROR(log(), "failed to find joint name %s", name.c_str());
        return info_.joints.end();
    }

    return res;
}

std::vector<hardware_interface::StateInterface>
ZoeMotorHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces{
        hardware_interface::StateInterface(
            getJointComponentByName("back_left_hinge_joint")->name,
            hardware_interface::HW_IF_VELOCITY, &backLeftVel),
        hardware_interface::StateInterface(
            getJointComponentByName("back_right_hinge_joint")->name,
            hardware_interface::HW_IF_VELOCITY, &backRightVel),
        hardware_interface::StateInterface(
            getJointComponentByName("front_left_hinge_joint")->name,
            hardware_interface::HW_IF_VELOCITY, &frontLeftVel),
        hardware_interface::StateInterface(
            getJointComponentByName("front_right_hinge_joint")->name,
            hardware_interface::HW_IF_VELOCITY, &frontRightVel)};

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ZoeMotorHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        getJointComponentByName("back_left_hinge_joint")->name,
        hardware_interface::HW_IF_VELOCITY, &backLeftCmdVel));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        getJointComponentByName("back_right_hinge_joint")->name,
        hardware_interface::HW_IF_VELOCITY, &backRightCmdVel));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        getJointComponentByName("front_left_hinge_joint")->name,
        hardware_interface::HW_IF_VELOCITY, &frontLeftCmdVel));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        getJointComponentByName("front_right_hinge_joint")->name,
        hardware_interface::HW_IF_VELOCITY, &frontRightCmdVel));

    // };

    return command_interfaces;
}

hardware_interface::CallbackReturn ZoeMotorHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ZoeMotorHardware"),
                "Configuring ...please wait...");
    // TODO: setup comms

    connection = std::make_shared<Command>(CAN_INTERFACE, true);
    if(!connection->checkOpenResult()) {
        return hardware_interface::CallbackReturn::ERROR;
    } 
    if(connection->setOperational(CANOPEN_ID_1) < 0) {
        return hardware_interface::CallbackReturn::ERROR;
    } 
    if(connection->setOperational(CANOPEN_ID_2) < 0) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    if(connection->setOperational(CANOPEN_ID_3) < 0) {
        return hardware_interface::CallbackReturn::ERROR;
    } 
    if(connection->setOperational(CANOPEN_ID_4) < 0) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (connection->testCan(CANOPEN_ID_1) < 0) {
        std::cout << "Echo 1 failed" << std::endl;
        return hardware_interface::CallbackReturn::ERROR;
    } 
    if (connection->testCan(CANOPEN_ID_2) < 0) {
        std::cout << "Echo 2 failed" << std::endl;
        return hardware_interface::CallbackReturn::ERROR;
    } 
    if (connection->testCan(CANOPEN_ID_3) < 0) {
        std::cout << "Echo 3 failed" << std::endl;
        return hardware_interface::CallbackReturn::ERROR;
    } 
    if (connection->testCan(CANOPEN_ID_4) < 0) {
        std::cout << "Echo 4 failed" << std::endl;
        return hardware_interface::CallbackReturn::ERROR;
    } 


    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZoeMotorHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ZoeMotorHardware"),
                "Cleaning up ...please wait...");

    // TODO: cleanup comms

    RCLCPP_INFO(rclcpp::get_logger("ZoeMotorHardware"),
                "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZoeMotorHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ZoeMotorHardware"),
                "Activating ...please wait...");
    // TODO:  setup pid and initial config to driver

    connection->setLimits(-320000, 320000, -320000, 320000, CANOPEN_ID_1);
    connection->setLimits(-320000, 320000, -320000, 320000, CANOPEN_ID_2);
    connection->setLimits(-320000, 320000, -320000, 320000, CANOPEN_ID_3);
    connection->setLimits(-320000, 320000, -320000, 320000, CANOPEN_ID_4);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZoeMotorHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ZoeMotorHardware"),
                "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("ZoeMotorHardware"),
                "Successfully deactivated!");

    connection->stop();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
ZoeMotorHardware::read(const rclcpp::Time & /*time*/,
                       const rclcpp::Duration &period) {
    //   TODO: read encoder values

    int speed_1;
    int speed_2;
    int speed_3;
    int speed_4;
    connection->getSpeed(&speed_1, CANOPEN_ID_1);
    connection->getSpeed(&speed_2, CANOPEN_ID_2);
    connection->getSpeed(&speed_3, CANOPEN_ID_3);
    connection->getSpeed(&speed_4, CANOPEN_ID_4);
    std::cout << "speed = " << speed_1 << " " << speed_2 << " " << speed_3 << " " << speed_4 << std::endl;

    return hardware_interface::return_type::OK;
}



hardware_interface::return_type zoe_motor_hardware::ZoeMotorHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    
    int backLeftVel = connection->get_counts(backLeftCmdVel);
    int backRightVel = connection->get_counts(backRightCmdVel);
    int fronLefttVel = connection->get_counts(frontLeftCmdVel);
    int frontRightVel = connection->get_counts(frontRightCmdVel);
    
    connection->setSpeed(backLeftVel, CANOPEN_ID_1);
    connection->setSpeed(backRightVel, CANOPEN_ID_2);
    connection->setSpeed(fronLefttVel, CANOPEN_ID_3);
    connection->setSpeed(frontRightVel, CANOPEN_ID_4);
    return hardware_interface::return_type::OK;
}

} // namespace zoe_motor_hardware

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(diffdrive_arduino::ZoeMotorHardware,
//                        hardware_interface::SystemInterface)

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(zoe_motor_hardware::ZoeMotorHardware,
                            hardware_interface::SystemInterface)