#include "zoe2_hardware/rs232.hpp"
#include "zoe2_hardware/can.hpp"
#include <memory>


#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using FuncCode = zoe2_hardware::Dispatcher::CANFunctionCode;

#ifndef ZOE_COMMAND
#define ZOE_COMMAND

enum Mode
{
	MODE_POS = 5,   /** Position mode: Motor is to be controlled with a position command. */
	MODE_TORQUE = 1, /** Torque mode: Motor is to be controlled with a torque command. */
    MODE_SPEED = 2
};

enum open_result_t {
    OPEN_FAILED = -20,
    OPEN_SUCCESS = 20
};

enum can_baudrate_t {
    CAN_0 = 1000000,
    CAN_1 = 500000,
    CAN_2 = 250000,
    CAN_3 = 125000,
    CAN_4 = 100000,
    CAN_5 = 50000,
    CAN_6 = 50000,
    CAN_7 = 50000,
    CAN_8 = 800000
};


class Command {
    public:
        Command(const std::string& iface, bool is_can) {
            if(is_can) {
                printf("Inside constructor CAN\n");
                tcan_ = std::make_unique<TCan>(iface);
                rs232_ = nullptr;
                int res = tcan_->open();
                if (open_res_ < 0) {
                    printf("CAN_OPEN: %d\n", res);
                    open_res_ = OPEN_FAILED;  
                } else {
                    printf("CAN_OPEN SUCCESS: %d\n", res);
                    open_res_ = OPEN_SUCCESS;
                }
            } else {
                rs232_ = std::make_unique<TRs232>(iface); 
                tcan_ = nullptr;
                int res = rs232_->serial_open();
                int res_setup = rs232_->setup();
                if (res < 0 || res_setup < 0) {
                    open_res_ = OPEN_FAILED;
                } else {
                    open_res_ = OPEN_SUCCESS;
                }
            }
        }


        int close_port();

        int send(const int size, const std::string& cmd, unsigned int can_id);
        int receive(unsigned char *output, unsigned int can_id, FuncCode FCode);
        int receive(struct can_frame& frame, unsigned int can_id, FuncCode FCode);

        bool checkOpenResult();
        int setOperational(unsigned int can_id = -1); 
        int nmtStart(unsigned int can_id = -1);
        int nmtStop(unsigned int can_id = -1);
        
        // Commands implemented

        int setCanAddress(int can_address, unsigned int can_id = -1);
        int setCanBaudRate(can_baudrate_t baud_rate, unsigned int can_id = -1);
        int saveParametersToFlash(unsigned int can_id = -1);

        int testCan(unsigned int can_id);

        int setPosition(int pos, unsigned int can_id = -1);
        int getPosition(int* pos, unsigned int can_id = -1);
        
        int setForce(float force, unsigned int can_id = -1);
        // int getForce(float* force,unsigned int id = -1);

        int startMotor(unsigned int can_id = -1);
        int stopMotor(unsigned int can_id = -1);

        int beginMotion(unsigned int can_id = -1);
        int stop(unsigned int can_id = -1);

        int setUnitMode(int mode, unsigned int can_id = -1);
        
        int setSpeedPTP(int speed, unsigned int can_id = -1);
        int setVelocity(int speed, unsigned int can_id = -1);
        
        int setSpeed(int speed, unsigned int can_id = -1); 
        int configureSpeedMode(unsigned int can_id = -1);
        int getSpeed(int *speed, unsigned int can_id = -1);

        int setAbsolutePosition(int pos, unsigned int can_id = -1);
        int setRelativePosition(int pos, unsigned int can_id = -1);
        
        int setTorque(float torque, unsigned int can_id = -1);

        int getMaxCurrent(float* current, unsigned int can_id = -1);
        int getActiveCurrent(int* current, unsigned int can_id = -1);

        int setLimits(int vmin, int vmax, int fmin, int fmax, unsigned int can_id = -1);

        int intFromData(unsigned char* data);
        float floatFromData(unsigned char* data);

        int intFromDataBigEndian(unsigned char* data, int start_index, int length);

        unsigned int get_can_id();

        int get_counts(double speed);
        double get_speed_counts(int count);



        int getSocketFD() const{
            return tcan_->getSocket();
        }

        void setDispatcher(std::shared_ptr<zoe2_hardware::Dispatcher> dispatcher){
            if(tcan_){
                tcan_ -> setDispatcher(dispatcher);
            }
        }


        private:
            std::unique_ptr<TRs232> rs232_;
            std::unique_ptr<TCan> tcan_;
            open_result_t open_res_; 

};

#endif