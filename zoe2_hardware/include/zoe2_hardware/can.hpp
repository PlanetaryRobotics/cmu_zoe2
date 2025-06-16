#include <string>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <memory>
#include "zoe2_hardware/dispatcher.hpp"

#ifndef ZOE_CAN
#define ZOE_CAN

struct can_frame;

enum ParseResultType {
  Integer,
  Float,
  Invalid
};


struct ParseValue {
  ParseResultType type;
  union {
    float float_val;
    int int_val;
  };
};

class TCan {

public:

  TCan(const std::string& iface);
  ~TCan();
  
  int open();

  int setOperational(unsigned int can_id);
  int setPreOperational(unsigned int can_id);
  int nmtStart(unsigned int can_id);

  int sendMsg(int size, const std::string& cmd, unsigned int can_id);
  int receiveMsg(unsigned char *output, unsigned int can_id);
  int receiveMsg(struct can_frame& frame, unsigned int can_id);

  int sendMsgDiscardReply(int size, const std::string& cmd, unsigned int can_id);

  static void createFrame(struct can_frame& frame, int id, int len, unsigned char* data);

  void setData(unsigned char* data, int i);
  void setData(unsigned char* data, float f);
  void setIndex(unsigned char* data, int index);
  float floatFromData(unsigned char* data);
  int intFromData(unsigned char* data);

  int getSocket() const;

  std::shared_ptr<zoe2_hardware::Dispatcher> dispatcher_;

  void setDispatcher(std::shared_ptr<zoe2_hardware::Dispatcher> dispatcher);

private:

  std::string iface_;
  struct sockaddr_can addr_;
  struct ifreq ifr_;
  int socket_;

  int sendFrame(struct can_frame& frame);
  void createCmd(const std::string& input, unsigned char* output, const int size);
  bool parseCommand(const std::string& cmd, std::string& command, int& index, ParseValue& value);

  

};

#endif // ZOE_TCAN_HPP