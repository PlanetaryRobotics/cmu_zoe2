#pragma once

#include <thread>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <array>
#include <atomic>
#include <linux/can.h>
#include <fcntl.h>
#include <deque>




namespace zoe2_hardware {

class Dispatcher {
public:
    Dispatcher(int socket_fd);
    ~Dispatcher();

    void start();  // Starts the background thread
    void stop();   // Signals thread to exit and joins it

    std::vector<can_frame> getMessagesForId(uint32_t can_id);

private:
    void run();

    int socket_fd_;
    std::thread thread_;
    std::atomic<bool> running_;

    std::unordered_map<uint32_t, std::deque<can_frame>> buffer_; 
    std::mutex buffer_mutex_;                                                  
    const size_t max_buffer_size = 3;
};

} // namespace zoe2_hardware
