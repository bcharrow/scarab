#ifndef _ACRONAME_MOTOR_H_
#define _ACRONAME_MOTOR_H_

#include <string>
#include <aCommon/aStem.h>    /* stem library header   */
#include <aCommon/aMotion.h> /* motion control library */
#include <stdlib.h>

#define aMODULE    4

using namespace std;

class AcronameMotor
{
private:
  int left_channel, right_channel;
  string portname;
  long portspeed;

  aErr error;
  aIOLib ioLib;
  aStemLib stemLib;
  aStreamRef linkStream;

public:
  AcronameMotor();
  ~AcronameMotor();
  void SetupPort(const string &portname, const long &portspeed);
  void SetupChannels(int left, int left_dir, int right, int right_dir);
  int SetPWMFreq(unsigned char paramH, unsigned char paramL);
  void SetupPID(double p, double i, double d, double period);

  int SetVel(short int left_vel, short int right_vel);
  int GetVel(short int &left_vel, short int &right_vel);
  
  bool ok();
};

#endif
