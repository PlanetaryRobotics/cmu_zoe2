#include "zoe2_hardware/commander.hpp"
#include <memory>

#define ENCODER_PPR 1024

#define PI 3.14


bool Command::checkOpenResult() {
  if(open_res_ == OPEN_FAILED) {
    return false;
  } else {
    return true;
  }
}

int Command::setOperational(unsigned int can_id) {
  if(rs232_ != nullptr) {
    return rs232_->setOperational();
  } else if(tcan_ != nullptr) {
    return tcan_->setOperational(can_id);
  } else {
    return -1000;
  }
}

int Command::nmtStart(unsigned int can_id) {
  if(rs232_ != nullptr) {
    return -1;
  } else if(tcan_ != nullptr) {
    return tcan_->nmtStart(can_id);
  } else {
    return -1000;
  }
}


int Command::send(const int size, const std::string& cmd, unsigned int can_id) {
  if(rs232_ != nullptr) {
    return rs232_->sendMsg(cmd);
  } else if(tcan_ != nullptr) {
    return tcan_->sendMsg(size, cmd, can_id);
  } else {
    return -1000;
  }

}

int Command::receive(unsigned char *output, unsigned int can_id) {
  if(rs232_ != nullptr) {
    return rs232_->receiveMsg(output);
  } else if(tcan_ != nullptr) {
    return tcan_->receiveMsg(output, can_id);
  } else {
    return -1000;
  }
}


int Command::receive(struct can_frame& frame, unsigned int can_id) {
    return tcan_->receiveMsg(frame, can_id);
}

int Command::sendMsgDiscardReply(const int size, const std::string& cmd, unsigned int can_id) {
  if(rs232_ != nullptr) {
    return rs232_->sendMsgDiscardReply(cmd);
  } else if(tcan_ != nullptr) {
    return tcan_->sendMsgDiscardReply(size, cmd, can_id);
  } else {
    return -1000;
  }
}

int Command::get_counts(double speed) {
  return int(speed * ENCODER_PPR / (2 * PI));
}

double Command::get_speed_counts(int counts) {
  return ((double)counts * 2 * PI / ENCODER_PPR);
}

// unsigned int Command::get_can_id() {
//   return tcan_->get_can_id();
// }

int Command::testCan(unsigned int can_id) {
    can_frame frame;
    std::string mess = "SN[2]";

    if(send(4, mess, can_id) < 0) {
      return -1;
    }

    printf("SENT MESSAGE!!\n");

    if(receive(frame, can_id) < 0) {
      return -2;
    }

    printf("RECEIVE MESSAGE!!\n");


    if (frame.can_id != (can_id | (5 << 7))) return -4; /* RPDO COB-ID: 0x281-0x2ff */
    if (frame.can_dlc != 8) return -5;
    if (frame.data[0] != 0x53) return -6;
    if (frame.data[1] != 0x4e) return -6;
    if (frame.data[2] != 0x02) return -6;
    if (frame.data[3] != 0x00) return -6;
    if (frame.data[4] != 0x35) return -6;
    if (frame.data[5] != 0x01) return -6;
    if (frame.data[6] != 0x03) return -6;
    if (frame.data[7] != 0x00) return -6;


  return 0;

}


int Command::beginMotion(unsigned int can_id) {
  std::string msg = "BG";
  return sendMsgDiscardReply(4, msg, can_id);
}

// To be used with RS232 for CAN setup
int Command::setCanAddress(int can_address, unsigned int can_id) {
  std::string msg = "PP[13]=" + std::to_string(can_address);
  return sendMsgDiscardReply(8, msg, can_id);
}

// To be used with RS232 for CAN setup
int Command::setCanBaudRate(can_baudrate_t baud_rate, unsigned int can_id) {
  std::string msg = "PP[14]=" + std::to_string(baud_rate);
  return sendMsgDiscardReply(8, msg, can_id);
}

// To be used with RS232 for CAN setup
int Command::saveParametersToFlash(unsigned int can_id) {
  std::string msg = "SV";
  return sendMsgDiscardReply(4, msg, can_id);
}

int Command::stopMotor(unsigned int can_id) {
  std::string msg = "MO=0";
  return sendMsgDiscardReply(8, msg, can_id); 
}

int Command::stop(unsigned int can_id) {
	stopMotor(can_id);
	setUnitMode(MODE_POS, can_id); /* UnitMode must be MODE_POS for ST to work. */
  std::string msg = "ST";
	return sendMsgDiscardReply(4, msg, can_id);
}

int Command::setUnitMode(int mode, unsigned int can_id) {
  std::string msg = "UM=" + std::to_string(mode);
  stopMotor(can_id);
	return sendMsgDiscardReply(8, msg, can_id);
}

int Command::startMotor(unsigned int can_id) {
  std::string msg = "MO=1";
  return sendMsgDiscardReply(8, msg, can_id);
}

int Command::setVelocity(int speed, unsigned int can_id) {
  std::string msg = "JV=" + std::to_string(speed);
  return sendMsgDiscardReply(8, msg, can_id);
}

int Command::setSpeedPTP(int speed, unsigned int can_id) {
  std::string msg = "SP=" + std::to_string(speed);
  return sendMsgDiscardReply(8, msg, can_id);
}


int Command::setAbsolutePosition(int pos, unsigned int can_id) {
  std::string data = "PA=" + std::to_string(pos);
	return sendMsgDiscardReply(8, data, can_id);}

int Command::setRelativePosition(int pos,unsigned int can_id) {
  std::string data = "PR=" + std::to_string(pos);
	return sendMsgDiscardReply(8, data, can_id);
}

int Command::setTorque(float torque, unsigned int can_id) {
  std::string data = "TC=" + std::to_string(torque);
	return sendMsgDiscardReply(8, data, can_id);
}

int Command::getMaxCurrent(float* current, unsigned int can_id) {
	can_frame frame;
  std::string msg = "MC";
	int rval;
	
	rval = send(4, msg, can_id);
	if (rval < 0) {
		return rval;
	}

	rval = receive(frame, can_id);
	if (rval < 0) {
		return rval;
	}

	*current = floatFromData(frame.data);
	return 0;
}

int Command::setLimits(int vmin, int vmax, int fmin, int fmax, unsigned int can_id) {

	stopMotor(can_id); /* The motor must be stopped before any changes in the unit mode can be made. */ 
	setUnitMode(MODE_POS, can_id); /* Again the damn LL and HL commands work only in MODE_POS. */
  std::string data1 = "VL[2]=" + std::to_string(vmin);
	sendMsgDiscardReply(8, data1, can_id);
  std::string data2 = "VH[2]=" + std::to_string(vmax);
	sendMsgDiscardReply(8, data2, can_id);
  std::string data3 = "LL[2]=" + std::to_string(fmin);
	sendMsgDiscardReply(8, data3, can_id);
  std::string data4 = "HL[2]=" + std::to_string(fmax);
	sendMsgDiscardReply(8, data4, can_id);
	return 0;
}


int Command::setPosition(int pos, unsigned int can_id) {
  int rval;

  rval = stopMotor(can_id);
  rval |= setUnitMode(MODE_POS, can_id);
  rval |= startMotor(can_id);
  rval |= setAbsolutePosition(pos, can_id);
  rval |= beginMotion(can_id);

  return rval;
}

int Command::setSpeed(int speed, unsigned int can_id) {
  int rval=0;
  rval |= setVelocity(speed, can_id);
  rval |= beginMotion(can_id);
  return rval;

}

int Command::configureSpeedMode(unsigned int can_id) {
  int rval;
  rval = stopMotor(can_id);
  rval |= setUnitMode(MODE_SPEED, can_id);
  rval |= startMotor(can_id);
  return rval;
}

int Command::getPosition(int* pos, unsigned int can_id) {
  
  can_frame frame;
  std::string data = "PX";
  
  if(send(4, data, can_id) < 0) {
    return -1;
  }

  if(receive(frame, can_id) < 0) {
    return -2;
  }

  *pos = intFromData(frame.data);
  return 0;

}


int Command::getSpeed(int* speed, unsigned int can_id) {
  
  can_frame frame;
  std::string data = "VX";
  
  if(send(4, data, can_id) < 0) {
    return -1;
  }

  if(receive(frame, can_id) < 0) {
    return -2;
  }

  *speed = intFromData(frame.data);
  return 0;

}

int Command::setForce(float force, unsigned int can_id) {

  int rval;
  
  rval = stopMotor(can_id);
  rval |= setUnitMode(MODE_TORQUE, can_id);
  rval |= startMotor(can_id);
  rval |= setTorque(force, can_id);

  return rval;

}

int Command::getForce(float* force, unsigned int can_id) {

  can_frame frame;
  std::string data = "IQ";
  
  if(send(4, data, can_id) < 0) {
    return -1;
  }

  if(receive(frame, can_id) < 0) {
    return -2;
  }

  *force = floatFromData(frame.data);
  return 0;

}

float Command::floatFromData(unsigned char* data) {

  int i = 0x00;
  
  i |= (data[7] << 24);
  i |= (data[6] << 16);
  i |= (data[5] << 8);
  i |= (data[4]);

  return *(float*)&i;
}

int Command::intFromData(unsigned char* data) {
  
  int i = 0x00;
  
  i |= (data[7] << 24);
  i |= (data[6] << 16);
  i |= (data[5] << 8);
  i |= (data[4]);

  return i;
}