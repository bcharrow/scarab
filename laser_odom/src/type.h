#ifndef TYPE_H
#define TYPE_H

#include "point2d.h"

#define PI 3.14159265

struct scanData
{
  double r;
  double theta;
  double alpha;
  double beta;
  point2d_t pt;
};

#endif
