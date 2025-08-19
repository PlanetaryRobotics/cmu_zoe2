#include "zoe2_hardware/rs232.hpp"
#include <fcntl.h>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <memory>


TRs232::TRs232(const std::string& port) : port_(port) {}

TRs232::~TRs232() {
    printf("Destrucing rs232!!\n");
    close(fd_);
    printf("Destrucing rs232 done!!\n");
}

int TRs232::serial_open() {
    fd_ = open(port_.c_str(), O_RDWR);

    if(fd_ < 0) {
        int err = errno;
        std::printf("Error %d from open: %s\n", err, strerror(err));
        return ERROR_CODE;
    }

    return fd_;
}

int TRs232::setup() {
    if(tcgetattr(fd_, &tty_) != 0) {
        int err = errno;
        std::printf("Error %d from tcgetattr: %s\n", err, strerror(err));
        return ERROR_CODE;
    }

    tty_.c_cflag &= ~PARENB;
    tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= CS8;
    tty_.c_cflag |= CREAD | CLOCAL; 
    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~ECHO; // Disable echo
    tty_.c_lflag &= ~ECHOE; // Disable erasure
    tty_.c_lflag &= ~ECHONL; // Disable new-line echo
    tty_.c_lflag &= ~ISIG; 
    tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty_.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty_.c_oflag &= ~ONLCR;
    tty_.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty_.c_cc[VMIN] = 0;

    if(tcsetattr(fd_, TCSANOW, &tty_) != 0) {
        int err = errno;
        std::printf("Error %d from tcsetattr: %s\n", err, strerror(err));
        return ERROR_CODE;
    }

    return SUCCESS_CODE;
}

int TRs232::setOperational() {
    return SUCCESS_CODE;
}

int TRs232::sendMsg(const std::string& msg) {
    unsigned char bytes[msg.length()];

    strToBytes(msg, bytes);

    ssize_t result = write(fd_, bytes, msg.length());

    // std::cout << "Bytes: " << std::endl;
    // for(size_t i = 0; i < msg.length(); i++) {
    //     std::cout << (int)bytes[i] << " ";
    // }
    // std::cout << std::endl;

    if(result < 0) {
        return ERROR_CODE;
    }

    return SUCCESS_CODE;

}

int TRs232::receiveMsg(unsigned char *output) {
    while(true) {
        ssize_t num_bytes = read(fd_, output, sizeof(output));
        
        if (num_bytes < 0) {
            printf("Error reading: %s", strerror(errno));
            return ERROR_CODE;
        }
        return SUCCESS_CODE;
    }
}

void TRs232::strToBytes(const std::string& input, unsigned char* output) {
    int i = 0;
    for(char c: input) {
        output[i++] = static_cast<unsigned char>(c);
        printf("%d ", output[i-1]);
    }
}

