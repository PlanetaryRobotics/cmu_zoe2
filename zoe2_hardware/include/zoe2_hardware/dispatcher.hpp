#pragma once

#include <thread>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <array>
#include <atomic>
#include <linux/can.h>
#include <fcntl.h>

namespace zoe2_hardware {

class Dispatcher {
public:
    explicit Dispatcher(int socket_fd);
    ~Dispatcher();

    void start();  // Starts the background thread
    void stop();   // Signals thread to exit and joins it

    //std::vector<std::array<uint8_t, 8>> getMessagesForId(uint32_t can_id);

private:
    void run();

    int socket_fd_;
    std::thread thread_;
    std::atomic<bool> running_;

    std::unordered_map<uint32_t, std::vector<std::array<uint8_t, 8>>> buffer_;
    std::mutex buffer_mutex_;
};

} // namespace zoe_motor_hardware
