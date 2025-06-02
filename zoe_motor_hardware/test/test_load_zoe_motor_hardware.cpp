// test_load_my_hardware.cpp

#include <gmock/gmock.h>
#include <controller_manager/controller_manager.hpp>

TEST(LoadingMyHardwareInterface, load) {
  controller_manager::ControllerManager cm;
  cm.load_controller("zoe_motor_hardware"); 
}