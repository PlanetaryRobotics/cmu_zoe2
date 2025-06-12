#include "zoe2_hardware/dispatcher.hpp"
#include <unistd.h> // for read()
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace zoe2_hardware {

Dispatcher::Dispatcher(int /*socket_fd*/){}

Dispatcher::~Dispatcher() {
    stop();
}

void Dispatcher::start() {
    running_ = true;
    thread_ = std::thread(&Dispatcher::run, this);

}

void Dispatcher::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void Dispatcher::run() {
    
//    /*
        while(running_){
        std::cout<<"working"<<std::endl;
        RCLCPP_INFO(rclcpp::get_logger("Dispatcher"),"working!");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
// */
     /*
    struct can_frame frame;
    while (running_) {
        can_frame frame;
        int result = read(can_->getSocket(), &frame, sizeof(frame));
        if (result > 0) {
            std::cout << "[Dispatcher] CAN ID: 0x" << std::hex << frame.can_id
                      << " Data: ";
            for (int i = 0; i < frame.can_dlc; ++i) {
                std::cout << std::setw(2) << std::setfill('0')
                          << std::hex << static_cast<int>(frame.data[i]) << " ";
            }
            std::cout << std::dec << std::endl;
        }
*/
}}
/*
std::vector<std::array<uint8_t, 8>> Dispatcher::getMessagesForId(uint32_t can_id) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_[can_id];  // returns a copy
}*/

 // namespace zoe_motor_hardware
