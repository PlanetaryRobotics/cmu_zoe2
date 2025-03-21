
/**
 * This is a test file to test the CAN communications
*/

#include <iostream>
#include <chrono>
#include <thread>
#include "zoe_motor_hardware/commander.hpp"
#include <memory>

#define CAN_INTERFACE "can0"
#define CANOPEN_ID_1 1
#define CANOPEN_ID_2 2
#define CANOPEN_ID_3 3
#define CANOPEN_ID_4 4

void print_info(std::shared_ptr<Command> elmo, unsigned int can_id = -1) {
    int position;
    while (true) {
        elmo->getSpeed(&position, can_id);
        std::cout << "ID = " << can_id << "speed = " << position << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
void test_speed(std::shared_ptr<Command> elmo, unsigned int can_id = -1) {
    elmo->setSpeed(20000, can_id);
    print_info(elmo, can_id);
}

void stop(std::shared_ptr<Command> elmo, unsigned int can_id = -1) {
    elmo->stop(can_id);
}



void test(std::shared_ptr<Command> elmo, unsigned int can_id = -1) {
    elmo->setLimits(-320000, 320000, -320000, 320000, can_id);
    test_speed(elmo, can_id);
}


int bleh() {
    std::shared_ptr<Command> can = std::make_shared<Command>(CAN_INTERFACE, true);

    if (!can->checkOpenResult()) {
        std::cout << "CanOpen 1 failed" << std::endl;
        return EXIT_FAILURE;
    } else {
        int res = can->setOperational(CANOPEN_ID_1);
        res = can->setOperational(CANOPEN_ID_2);
        res = can->setOperational(CANOPEN_ID_3);
        res = can->setOperational(CANOPEN_ID_4);
        if(res < 0) {
            std::cout << "CanOpen 1 failed" << std::endl;
            return EXIT_FAILURE;
        }
    }
    
    if (can->testCan(CANOPEN_ID_1) < 0) {
        std::cout << "CanOpen 1 failed" << std::endl;
        return EXIT_FAILURE;
    }


    

    std::cout << "INITAL TESTS DONE!!!!!!!!!!!!!!" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    test(can, CANOPEN_ID_1);
    test(can, CANOPEN_ID_2);
    test(can, CANOPEN_ID_3);
    test(can, CANOPEN_ID_4);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    stop(can, CANOPEN_ID_1);
    stop(can, CANOPEN_ID_2);
    stop(can, CANOPEN_ID_3);
    stop(can, CANOPEN_ID_4);

    std::cout << "CAN test end" << std::endl;
    return EXIT_SUCCESS;
}

int main() {
    return bleh();
}

