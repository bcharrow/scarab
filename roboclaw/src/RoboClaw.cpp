#include "RoboClaw.h"

#include <vector>
#include <ros/ros.h>

using namespace std;

//! Macro for throwing an exception with a message, passing args
#define SERIAL_EXCEPT(msg, ...)                                         \
  {                                                                     \
    char buf[1000];                                                     \
    snprintf(buf, 1000, msg " (in USBSerial::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw std::runtime_error(buf);                                      \
  }

void USBSerial::Open(const char *port) {
  if (IsOpen()) {
    Close();
  }

  fd_ = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);
  buf_start_ = buf_end_ = 0;
  if (fd_ == -1) {
    SERIAL_EXCEPT("Failed to open port: %s. %s (errno = %d)",
                  port, strerror(errno), errno);
  }

  try {
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    fl.l_pid = getpid();

    if (fcntl(fd_, F_SETLK, &fl) != 0) {
      SERIAL_EXCEPT("Device %s is already locked", port);
    }

    struct termios newtio;
    tcgetattr(fd_, &newtio);
    memset(&newtio.c_cc, 0, sizeof(newtio.c_cc)); // Clear special characters
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) < 0) {
      SERIAL_EXCEPT("Couldn't set serial port attributes: %s", port);
    }

    usleep(200000);
    Flush();
  } catch (std::runtime_error &e) {
    if (fd_ != -1) {
      close(fd_);
    }
    fd_ = -1;
    throw e;
  }
}

bool USBSerial::IsOpen() {
  return fd_ != -1;
}

void USBSerial::Close() {
  if (IsOpen()) {
    Flush();
  }
  int rv = close(fd_);
  if (rv != 0) {
    SERIAL_EXCEPT("Error closing port: %s (%d)", strerror(errno), errno);
  }
}

int USBSerial::Flush() {
  int retval = tcflush(fd_, TCIOFLUSH);
  if (retval != 0)
    SERIAL_EXCEPT("tcflush failed");
  buf_start_ = 0;
  buf_end_ = 0;
  return retval;
}

int USBSerial::Write(const unsigned char *data, int len) {
  int origflags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK);
  // fprintf(stderr, "Writing ");
  // for (int i = 0; i < len; ++i) {
  //   fprintf(stderr, "%02x ", data[i]);
  // }
  // fprintf(stderr, "\n");
  ssize_t retval = write(fd_, data, len);
  int fputserrno = errno;
  fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);
  errno = fputserrno;
  if (retval != -1 ) {
    return retval;
  } else {
    SERIAL_EXCEPT("write() failed: %s (%d)", strerror(errno), errno);
  }
}

int USBSerial::Read(char *buf, int len, int timeout, bool translate) {
  int current = 0;
  struct pollfd ufd[1];
  int retval;
  ufd[0].fd = fd_;
  ufd[0].events = POLLIN;

  while (true) {
    if (buf_start_ == buf_end_) {
      if ((retval = poll(ufd, 1, timeout)) < 0) {
        SERIAL_EXCEPT("Poll failed %s (%d)", strerror(errno), errno);
      }
      else if (retval == 0) {
        SERIAL_EXCEPT("Timeout reached");
      }
      else if (ufd[0].revents & POLLERR) {
        SERIAL_EXCEPT("Error on socket");
      }
      int bytes = read(fd_, read_buf_, sizeof(read_buf_));
      // fprintf(stderr, "Got %i bytes\n  ", bytes);
      // for (int i = 0; i < bytes; ++i) {
      //   fprintf(stderr, " %02x", read_buf_[i]);
      // }
      // fprintf(stderr, "\n");
      buf_start_ = 0;
      buf_end_ = bytes;
    }

    while (buf_start_ != buf_end_) {
      if (translate) {
        if (current == len - 1) {
          buf[current] = 0;
          SERIAL_EXCEPT("Buffer filled without end of line being found");
        }
        buf[current] = read_buf_[buf_start_];
        buf_start_++;
        if (read_buf_[current++] == '\n') {
          buf[current] = 0;
          return current;
        }
      } else {
        buf[current] = read_buf_[buf_start_];
        buf_start_++;
        current++;
        if (current == len) {
          return current;
        }
      }
    }
  }
}

//
// Constructor
//
RoboClaw::RoboClaw(USBSerial *ser) {
  setSerial(ser);
}

//
// Destructor
//
RoboClaw::~RoboClaw() {

}


void RoboClaw::setSerial(USBSerial *ser) {
  if (ser == NULL || !ser->IsOpen()) {
    throw invalid_argument("RoboClaw needs open USBSerial device");
  }
  ser_ = ser;
}

void RoboClaw::write_n(uint8_t cnt, ... ) {
  static unsigned char buff[256];
  int ind = 0;
  uint8_t crc=0;

  // send data with crc
  va_list marker;
  va_start(marker, cnt);
  // fprintf(stderr, "Sending: ");
  for(uint8_t index=0; index < cnt; index++) {
    uint8_t data = va_arg(marker, int);
    // fprintf(stderr, "%02x ", data);
    crc += data;
    buff[ind++] = data;
  }
  // fprintf(stderr, "\n");
  va_end(marker);              /* Reset variable arguments.      */
  buff[ind++] = crc & 0x7F;
  ser_->Write(buff, cnt + 1);
}

void RoboClaw::write(uint8_t byte) {
  // fprintf(stderr, "Sending: 0x%02x\n", byte);
  ser_->Write(&byte, 1);
}

uint8_t RoboClaw::read() {
  char d[1];
  int rv = ser_->Read(d, sizeof(d), 100, false);
  if (rv != 1) {
    // fprintf(stderr, "Rv = %i\n", rv);
    throw std::runtime_error("RoboClaw::write_n() Didn't get byte from read()");
  } else {
    // fprintf(stderr, "Read 0x%02x\n", d[0]);
  }
  return static_cast<uint8_t>(d[0]);
}

void RoboClaw::ForwardM1(uint8_t address, uint8_t speed) {
  write_n(3,address,M1FORWARD,speed);
}

void RoboClaw::BackwardM1(uint8_t address, uint8_t speed) {
  write_n(3,address,M1BACKWARD,speed);
}

void RoboClaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage) {
  write_n(3,address,SETMINMB,voltage);
}

void RoboClaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage) {
  write_n(3,address,SETMAXMB,voltage);
}

void RoboClaw::ForwardM2(uint8_t address, uint8_t speed) {
  write_n(3,address,M2FORWARD,speed);
}

void RoboClaw::BackwardM2(uint8_t address, uint8_t speed) {
  write_n(3,address,M2BACKWARD,speed);
}

void RoboClaw::ForwardBackwardM1(uint8_t address, uint8_t speed) {
  write_n(3,address,M17BIT,speed);
}

void RoboClaw::ForwardBackwardM2(uint8_t address, uint8_t speed) {
  write_n(3,address,M27BIT,speed);
}

void RoboClaw::ForwardMixed(uint8_t address, uint8_t speed) {
  write_n(3,address,MIXEDFORWARD,speed);
}

void RoboClaw::BackwardMixed(uint8_t address, uint8_t speed) {
  write_n(3,address,MIXEDBACKWARD,speed);
}

void RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed) {
  write_n(3,address,MIXEDRIGHT,speed);
}

void RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed) {
  write_n(3,address,MIXEDLEFT,speed);
}

void RoboClaw::ForwardBackwardMixed(uint8_t address, uint8_t speed) {
  write_n(3,address,MIXEDFB,speed);
}

void RoboClaw::LeftRightMixed(uint8_t address, uint8_t speed) {
  write_n(3,address,MIXEDLR,speed);
}

uint32_t RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid) {
  uint8_t send[2] = {address, cmd};
  ser_->Write(send, 2);

  uint8_t crc = address;
  crc+=cmd;

  uint32_t value;
  uint8_t data = read();
  crc+=data;
  value=(uint32_t)data<<24;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<16;

  data = read();
  crc+=data;
  value|=(uint32_t)data<<8;

  data = read();
  crc+=data;
  value|=(uint32_t)data;

  data = read();
  crc+=data;
  if(status)
    *status = data;
  data = read();
  if(valid)
    *valid = ((crc&0x7F)==data);

  return value;
}

uint32_t RoboClaw::ReadEncM1(uint8_t address, uint8_t *status,bool *valid) {
  return Read4_1(address,GETM1ENC,status,valid);
}

uint32_t RoboClaw::ReadEncM2(uint8_t address, uint8_t *status,bool *valid) {
  return Read4_1(address,GETM2ENC,status,valid);
}

int32_t RoboClaw::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid) {
  return Read4_1(address,GETM1SPEED,status,valid);
}

int32_t RoboClaw::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid) {
  return Read4_1(address,GETM2SPEED,status,valid);
}

void RoboClaw::ResetEncoders(uint8_t address) {
  write_n(2,address,RESETENC);
}

bool RoboClaw::ReadVersion(uint8_t address, string *version) {
  version->resize(32);

  uint8_t crc;
  write(address);
  crc=address;
  write(GETVERSION);
  crc+=GETVERSION;

  for(uint8_t i=0;i<32;i++) {
    (*version)[i] = read();
    crc += (*version)[i];
    if ((*version)[i] == 0) {
      return (crc&0x7F) == read();
    }
  }
  return false;
}

uint16_t RoboClaw::Read2(uint8_t address,uint8_t cmd,bool *valid) {
  uint8_t crc;
  write(address);
  crc=address;
  write(cmd);
  crc+=cmd;

  uint16_t value;
  uint8_t data = read();
  crc+=data;
  value=(uint16_t)data<<8;

  data = read();
  crc+=data;
  value|=(uint16_t)data;

  data = read();
  if(valid)
    *valid = ((crc&0x7F)==data);

  return value;
}

uint16_t RoboClaw::ReadMainBatteryVoltage(uint8_t address,bool *valid) {
  return Read2(address,GETMBATT,valid);
}

uint16_t RoboClaw::ReadLogicBattVoltage(uint8_t address,bool *valid) {
  return Read2(address,GETLBATT,valid);
}

void RoboClaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage) {
  write_n(3,address,SETMINLB,voltage);
}

void RoboClaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage) {
  write_n(3,address,SETMAXLB,voltage);
}

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

void RoboClaw::SetM1Constants(uint8_t address, uint32_t kd, uint32_t kp, uint32_t ki, uint32_t qpps) {
  write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

void RoboClaw::SetM2Constants(uint8_t address, uint32_t kd, uint32_t kp, uint32_t ki, uint32_t qpps) {
  write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t RoboClaw::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid) {
  return Read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t RoboClaw::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid) {
  return Read4_1(address,GETM2ISPEED,status,valid);
}

void RoboClaw::DutyM1(uint8_t address, uint16_t duty) {
  write_n(4,address,M1DUTY,SetWORDval(duty));
}

void RoboClaw::DutyM2(uint8_t address, uint16_t duty) {
  write_n(4,address,M2DUTY,SetWORDval(duty));
}

void RoboClaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2) {
  write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

void RoboClaw::SpeedM1(uint8_t address, uint32_t speed) {
  write_n(6,address,M1SPEED,SetDWORDval(speed));
}

void RoboClaw::SpeedM2(uint8_t address, uint32_t speed) {
  write_n(6,address,M2SPEED,SetDWORDval(speed));
}

void RoboClaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2) {
  write_n(10,address,M1SPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

void RoboClaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed) {
  write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

void RoboClaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed) {
  write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

void RoboClaw::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2) {
  write_n(10,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

void RoboClaw::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag) {
  write_n(19,address,M1SPEEDDIST,SetDWORDval(speed2),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

void RoboClaw::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag) {
  write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

void RoboClaw::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag) {
  write_n(23,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2) {
  bool valid;
  uint16_t value = Read2(address,GETBUFFERS,&valid);
  if(valid) {
    depth1 = value>>8;
    depth2 = value;
  }
  return valid;
}

bool RoboClaw::ReadCurrents(uint8_t address, uint8_t &current1, uint8_t &current2) {
  bool valid;
  uint16_t value = Read2(address,GETCURRENTS,&valid);
  if(valid) {
    current1 = value>>8;
    current2 = value;
  }
  return valid;
}

void RoboClaw::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2) {
  write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

void RoboClaw::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag) {
  write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

void RoboClaw::DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel) {
  write_n(6,address,M1DUTY,SetWORDval(duty),SetWORDval(accel));
}

void RoboClaw::DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel) {
  write_n(6,address,M2DUTY,SetWORDval(duty),SetWORDval(accel));
}

void RoboClaw::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint16_t accel1, uint16_t duty2, uint16_t accel2) {
  write_n(10,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(accel1),SetWORDval(duty2),SetWORDval(accel2));
}

uint8_t RoboClaw::ReadError(uint8_t address,bool *valid) {
  uint8_t crc;
  write(address);
  crc=address;
  write(GETERROR);
  crc+=GETERROR;

  uint8_t value = read();
  crc+=value;

  if(valid)
    *valid = ((crc&0x7F)==read());
  else
    read();

  return value;
}

void RoboClaw::WriteNVM(uint8_t address) {
  write_n(2,address,WRITENVM);
}

void RoboClaw::SetPWM(uint8_t address, uint8_t resolution) {
  write_n(3, address, SETPWM, resolution);
}
