#include "RoboClaw.h"

#include <ros/ros.h>

using namespace std;

#define ADDRESS 0x80
// #define Kp 0x00010000
// #define Ki 0x00008000
// #define KD 0x00004000
// #define qpps 44000
#define KP 0x00009000
#define KI 0x00000250
#define KD 0x00001000
#define QPPS 300000
// #define qpps 24000

void basic() {
  ros::NodeHandle nh;
  int32_t val, prev_val = 0, diff = 0;
  uint8_t status;
  bool valid;
  unsigned int baud = 38400U;
  string port("/dev/ttyACM0");
  ASIOSerialDevice ser(port, baud);

  RoboClaw roboclaw(&ser);

  // string version;
  // roboclaw.ReadVersion(ADDRESS, &version);
  // uint32_t enc1= roboclaw.ReadEncM1(ADDRESS, &status, &valid);

  // // cout << "Version: " << version;
  // cout << "Enc M1: " << enc1 << endl;
  // return 0;
  roboclaw.SetPWM(ADDRESS, 0);
  roboclaw.SetM1Constants(ADDRESS,KD,KP,KI,QPPS);
  roboclaw.SetM2Constants(ADDRESS,KD,KP,KI,QPPS);
  // roboclaw.ResetEncoders(ADDRESS);
  // roboclaw.ReadEncM1(ADDRESS, &status, &valid);

  uint32_t desired = QPPS;
  // roboclaw.ForwardM2(ADDRESS, 127);
  // roboclaw.ReadError(ADDRESS, &valid);
  roboclaw.SpeedAccelM2(ADDRESS, desired / 4.0, desired);
  // roboclaw.ForwardM2(ADDRESS, 10);
  ros::Time start = ros::Time::now();
  if (valid) {
    printf("%u\n", val);
  }

  while (ros::ok()) {
    ros::Duration(0.1).sleep();

    try {
      val = roboclaw.ReadEncM2(ADDRESS, &status, &valid);
      diff = val - prev_val;
      prev_val = val;

      if (valid) {
        printf("Encoder count: %u", val);
        if (status & 1) {
          printf(" UNDERFLOW");
        }
        if ((status & 2) >> 1) {
          printf(" BACKWARDS");
        }
        if ((status & 4) >> 2) {
          printf(" OVERFLOW");
        }
        printf("\n");
      }

      // Stop after 1 revolution
      // if (val > 2.7 * 1.5 * 16 * 4 * 500) {
      //   break;
      // }

      val = roboclaw.ReadSpeedM2(ADDRESS, &status, &valid);
      if(valid){
        printf("Speed : %i (status = %x) %i\n", val, status, diff);
      }

      // if (abs(val  - desired) / (double)desired < 0.05) {
      //   break;
      // }

    } catch (boost::system::system_error &e) {
      break;
    }
  }
  ros::Duration total = ros::Time::now() - start;
  cout << total << endl;
  roboclaw.ForwardM1(ADDRESS, 0);
  roboclaw.ForwardM2(ADDRESS, 0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "enc_demo");
  ros::Time::init();

  basic();
}
