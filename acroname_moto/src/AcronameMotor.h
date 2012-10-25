#ifndef _ACRONAME_MOTOR_H_
#define _ACRONAME_MOTOR_H_

#include <string>
#include <iostream>
#include <aCommon/aStem.h>    /* stem library header   */
#include <aCommon/aMotion.h> /* motion control library */
#include <stdlib.h>

class AcronameMotor
{
 public:
  struct MotoAddr {
    int module;
    int channel;

    std::string String() const {
      std::stringstream ss;
      ss << "(" << module << ", " << channel << ")";
      return std::string(ss.str());
    }
  };

  AcronameMotor();
  ~AcronameMotor();
  void SetupPort(const std::string &portname, const long &portspeed);

  void SetupChannel(const MotoAddr &m, bool forward);
  void SetPWMFreq(const MotoAddr &m, unsigned char paramH, unsigned char paramL);
  void SetupPID(const MotoAddr &m, double p, double i, double d, double period);

  void SetVel(const MotoAddr &m, short int vel);
  void GetVel(const MotoAddr &m, short int *vel);
  
  bool ok();
 private:
  std::string portname;
  long portspeed;

  aErr error;
  aIOLib ioLib;
  aStemLib stemLib;
  aStreamRef linkStream;
};

#endif
