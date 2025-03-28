#include "zoe_motor_hardware/can.hpp"
#include <cstring>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <unistd.h>
#include <sstream>
#include <stdbool.h>

TCan::TCan(const std::string& iface) : iface_(iface) {}

int TCan::open() {
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(socket_ < 0) {
    return -1;
  }


  addr_.can_family = AF_CAN;

  strcpy(ifr_.ifr_name, iface_.c_str());


  if(ioctl(socket_, SIOCGIFINDEX, &ifr_) < 0) {
    return -2;
  }


  addr_.can_ifindex = ifr_.ifr_ifindex;

  if(bind(socket_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
    return -3;
  }


  return 1;
}

TCan::~TCan() {
  close(socket_);
}

// unsigned int TCan::get_can_id() {
//   return can_id;
// }

int TCan::setOperational(unsigned int can_id) {
  can_frame frame;
  unsigned char data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  createFrame(frame, can_id, 2, data);
  
  return sendFrame(frame);
}

int TCan::setPreOperational(unsigned int can_id) {
  can_frame frame;
  unsigned char data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  createFrame(frame, can_id, 2, data);

  return sendFrame(frame);
}


int TCan::sendFrame(can_frame& frame) {
  int bytes = write(socket_, &frame, sizeof(frame));
  if(bytes != sizeof(frame)) {
    return -1;
  }
  return 0;
}

void TCan::createFrame(can_frame& frame, int id, int len, unsigned char* data) {
  frame.can_id = id;
  frame.can_dlc = len;
  memcpy(frame.data, data, 8);
}

int TCan::sendMsg(int size, const std::string& cmd, unsigned int can_id) {
  can_frame frame;
  unsigned char data[size];
  createCmd(cmd, data, size);

  createFrame(frame, can_id | (6 << 7), size, data);
  return sendFrame(frame);

}

int TCan::receiveMsg(unsigned char *output, unsigned int can_id) {
  int bytes;
  can_frame frame;

  while(true) {
    bytes = read(socket_, &frame, sizeof(frame));

    if(bytes < 0) {
      perror("Read error");
      return -1;
    }

    if (frame.can_id == (can_id | 5 << 7)) {
      memcpy(output, frame.data, 8);
      return 0;
    }
  }
}

int TCan::receiveMsg(struct can_frame& frame, unsigned int can_id) {
  int bytes;

  while(true) {
    bytes = read(socket_, &frame, sizeof(frame));

    if(bytes < 0) {
      perror("Read error");
      return -1;
    }


    if (frame.can_id == (can_id | 5 << 7)) {
    // printf("DONE READING!!\n");

      return 0;
    }
  }
}


void TCan::createCmd(const std::string& input, unsigned char* output, const int size) {
    int index;
    ParseValue value;
    std::string command;
    if(parseCommand(input, command, index, value)) {
      int i = 0;
      while(i < size) {
        output[i++] = 0x00;
      }

      i = 0;
      for(char c : command) {
          output[i++] = static_cast<unsigned char>(c);
      }

        switch(value.type) {
          case Integer:
            setData(output, value.int_val);
            break;
          case Float:
            setData(output, value.float_val);
            break;
          default:
            break;
        }
        
        // printf("Index is %x \n", index);
        if(index != -1) {
          setIndex(output, index);
        }
    }
    
}

void TCan::setIndex(unsigned char* data, int index) {
  data[2] = index & 0xFF;
  data[3] = (index >> 8) & 0x3F;
  
}

bool TCan::parseCommand(const std::string& cmd, std::string& command, int& index, ParseValue& value) {

  index = -1;
  value.float_val = -1;

  // Parse command and index as before
  command = cmd.substr(0,2);

  std::size_t equal = cmd.find('=');
  std::size_t closeBracket = cmd.find(']');
  std::size_t openBracket = cmd.find('[');

  if(openBracket != std::string::npos && 
     closeBracket != std::string::npos) {

    std::string indexStr = cmd.substr(openBracket+1, closeBracket-openBracket-1);

    std::stringstream ss1(indexStr);

    if(!(ss1 >> index)) {
      return false;
    }

  }
  if(equal != std::string::npos) {
    std::string valueStr = cmd.substr(equal+1);
    std::istringstream int_ss(valueStr);
    int_ss >> value.int_val;

    if(int_ss.eof() && !int_ss.fail()) {
      value.type = Integer;
      return true;
    } 
    std::istringstream float_ss(valueStr);
    float_ss >> std::noskipws >> value.float_val;

    if(float_ss.eof() && !float_ss.fail()) {
      value.type = Float;
      return true;
    } else {
      return false;
    }
  }

  return true;
}

int TCan::sendMsgDiscardReply(int size, const std::string& cmd, unsigned int can_id) {
  int result = sendMsg(size, cmd, can_id);
  unsigned char data[8];
  receiveMsg(data, can_id);
  return result;
}

void TCan::setData(unsigned char* data, int i) {

  data[7] = (i >> 24);
  data[6] = ((i << 8) >> 24); 
  data[5] = ((i << 16) >> 24);
  data[4] = ((i << 24) >> 24);

}

void TCan::setData(unsigned char* data, float f) {

  int i = *(int*)&f;
  
  data[7] = (i >> 24);
  data[6] = ((i << 8) >> 24);
  data[5] = ((i << 16) >> 24); 
  data[4] = ((i << 24) >> 24);

  data[3] |= 1 << 7; 

}

float TCan::floatFromData(unsigned char* data) {

  int i = 0x00;
  
  i |= (data[7] << 24);
  i |= (data[6] << 16);
  i |= (data[5] << 8);
  i |= (data[4]);

  return *(float*)&i;
}

int TCan::intFromData(unsigned char* data) {
  
  int i = 0x00;
  
  i |= (data[7] << 24);
  i |= (data[6] << 16);
  i |= (data[5] << 8);
  i |= (data[4]);

  return i;
}





