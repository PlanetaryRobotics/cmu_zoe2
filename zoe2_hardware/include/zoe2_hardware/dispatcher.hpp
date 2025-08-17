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

    enum class CANFunctionCode : uint8_t {
        NMT             = 0x0,  // 0000
        SYNC            = 0x1,  // 0001
        EMCY            = 0x1,  // 0001 (shares code with SYNC)
        TIME            = 0x2,  // 0010
        TPDO1           = 0x3,  // 0011
        RPDO1           = 0x4,  // 0100
        TPDO2           = 0x5,  // 0101
        RPDO2           = 0x6,  // 0110
        TPDO3           = 0x7,  // 0111
        RPDO3           = 0x8,  // 1000
        TPDO4           = 0x9,  // 1001
        RPDO4           = 0xA,  // 1010
        TSDO            = 0xB,  // 1011
        RSDO            = 0xC,  // 1100
        HEARTBEAT       = 0xE   // 1110
    };

    Dispatcher(int socket_fd);
    ~Dispatcher();

    void start();  // Starts the background thread
    void stop();   // Signals thread to exit and joins it

    std::vector<can_frame> getMessagesWithCOB(uint32_t can_id, CANFunctionCode function_code);
    uint32_t calculateCOBId(uint32_t can_id, CANFunctionCode function_code);

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
