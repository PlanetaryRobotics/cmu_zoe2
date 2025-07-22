
#include <termio.h>
#include <string>
#include "zoe2_hardware/dispatcher.hpp"

#ifndef ZOE_RS232
#define ZOE_RS232


enum result_t {
    SUCCESS_CODE = 10,
    ERROR_CODE = -10
};

class TRs232 {

    public:
        TRs232(const std::string& port);
        ~TRs232();
        
        int serial_open();
        int setup();
        int sendMsg(const std::string& msg);
        int receiveMsg(unsigned char *output);

        int sendMsgDiscardReply(const std::string& msg);

        int setOperational();

    private:
        std::string port_;
        int fd_;
        termios tty_;

        void strToBytes(const std::string& input, unsigned char* output);
};

#endif
