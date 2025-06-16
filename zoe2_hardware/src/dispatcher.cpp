#include "zoe2_hardware/dispatcher.hpp"
#include <unistd.h> // for read()
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <poll.h> 

namespace zoe2_hardware {

Dispatcher::Dispatcher(int socket_fd): socket_fd_(socket_fd){}

Dispatcher::~Dispatcher() {
    stop();
}

void Dispatcher::start() {

    int flags = fcntl(socket_fd_,F_GETFL,0);

    if(flags==-1){
        RCLCPP_ERROR(rclcpp::get_logger("Dispatcher"),"failed to get socket flags %s", strerror(errno));
        return;
    }
    if(fcntl(socket_fd_,F_SETFL,flags|O_NONBLOCK)==-1){
        RCLCPP_ERROR(rclcpp::get_logger("Dispatcher"),"failed to set socket to non-blocking %s", strerror(errno));
        return;
    }




    running_ = true;
    thread_ = std::thread(&Dispatcher::run, this);

}

void Dispatcher::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}
/*
void Dispatcher::run() {

    struct can_frame frame;

    while (running_) {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            char data_str[24] = {};
            snprintf(data_str, sizeof(data_str), "%02X %02X %02X %02X %02X %02X %02X %02X",
                     frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                     frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

            RCLCPP_INFO(rclcpp::get_logger("Dispatcher"),
                        "Received CAN ID: 0x%03X Data: %s",
                        frame.can_id, data_str);
        } else if (nbytes == -1){
            
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(rclcpp::get_logger("Dispatcher"),
                         "Error reading from CAN socket: %s", strerror(errno));
        }}

        // Prevent CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
}*/

std::vector<can_frame> Dispatcher::getMessagesForId(uint32_t can_id){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto it = buffer_.find(can_id);
    if (it !=buffer_.end()){
        return std::vector<can_frame>(it->second.begin(), it->second.end());
    }
    
    return{};
    
}

void Dispatcher::run() {
    struct can_frame frame;

    struct pollfd fds;
    fds.fd = socket_fd_;
    fds.events = POLLIN;

    while (running_) {
        int ret = poll(&fds, 1, 100);  // wait 100ms for data

        if (ret > 0 && (fds.revents & POLLIN)) {
            ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
            if (nbytes > 0) {


                
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                auto& queue = buffer_[frame.can_id];
                queue.push_front(frame);
                if (queue.size() > max_buffer_size){
                    queue.pop_back();
                }
                
                // RCLCPP_INFO(
                // rclcpp::get_logger("TCan"),
                // "CAN ID: 0x%03X Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                // frame.can_id,
                // frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                // frame.data[4], frame.data[5], frame.data[6], frame.data[7]);




            } else if (nbytes == -1) {
                RCLCPP_ERROR(rclcpp::get_logger("Dispatcher"),
                             "Error reading from CAN socket: %s", strerror(errno));
            }
        } else if (ret == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("Dispatcher"),
                         "poll() error: %s", strerror(errno));
        }

        // Else, ret == 0 => timeout, nothing to do
    }
}









}
