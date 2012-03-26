#include "fisher.h"
#include <limits>

void cov_fisher(vector<scanData> data, double* cov)
{
  if (data.size() == 0)
    {
      cov[0] = std::numeric_limits<double>::max();
      cov[1] = std::numeric_limits<double>::max();
      cov[2] = std::numeric_limits<double>::max();
      return;
    }

  unsigned int k = 0;		//Subsample
  while (k < data.size()-1)
  {
    
    double diff_x = data[k].pt.x-data[k+1].pt.x;
    double diff_y = data[k].pt.y-data[k+1].pt.y;
    double d = sqrt(diff_x*diff_x + diff_y*diff_y);
    if (d > 0.10)
    {
      k++;
    }
    else
    {
      data.erase(data.begin()+k+1);      
    }
  }

  for (int k = 0; k < (int)data.size(); k++)	//Calculate normal orientation
  {
    if (k-1 >= 0 && k+1 <= (int)data.size())
    {
      double sum_dx = 0;
      double sum_dy = 0;
      bool invalid_flag = false;

      double d1 = sqrt(pow(data[k-1].pt.x-data[k].pt.x,2) + pow(data[k-1].pt.y-data[k].pt.y,2));
      if (d1 <= 0.20)
      {
        sum_dx += (data[k-1].pt.x-data[k].pt.x) / d1;
        sum_dy += (data[k-1].pt.y-data[k].pt.y) / d1;
      }
      else
        invalid_flag = true;

      double d2 = sqrt(pow(data[k].pt.x-data[k+1].pt.x,2) + pow(data[k].pt.y-data[k+1].pt.y,2));
      if (d2 <= 0.20)
      {
        sum_dx += (data[k].pt.x-data[k+1].pt.x) / d2;
        sum_dy += (data[k].pt.y-data[k+1].pt.y) / d2;
      }
      else
        invalid_flag = true;

      if (!invalid_flag)
      {
        data[k].alpha = atan2(sum_dy, sum_dx) + PI/2;
        data[k].beta = data[k].alpha - data[k].theta;
      }
        
    }
  }

  k = 0;	//Remove points that do not have normal orientation
  while (k < data.size())
  {
    if (data[k].alpha != 100)
      k++;
    else
      data.erase(data.begin()+k);
  }

  mat FIM(3,3);
  FIM.zeros();
  mat f(3,3);
  f.zeros();

  for (unsigned int k = 0; k < data.size(); k++)
  {
    if (!isnan(data[k].alpha))
    {
      f.zeros();
      double r = data[k].r;
      double c = cos(data[k].alpha);
      double s = sin(data[k].alpha);
      double z = 1 / cos(data[k].beta);
      double t = tan(data[k].beta);
      f(1-1,1-1) = c*c*z*z;
      f(2-1,1-1) = c*s*z*z;
      f(3-1,1-1) = c*z*t*r;
      f(1-1,2-1) = c*s*z*z;
      f(2-1,2-1) = s*s*z*z;
      f(3-1,2-1) = s*z*t*r;
      f(1-1,3-1) = c*z*t*r;
      f(2-1,3-1) = s*z*t*r;
      f(3-1,3-1) = t*r*t*r;
      FIM = FIM + f; 
    }
  }
  FIM = FIM / 0.02;	//Normalize with sensor noise covariance: 0.02

  if (det(FIM) < 1e-6) {
    ROS_WARN("FIM has small determinant");
    cov[0] = -1;
    cov[1] = -1;
    cov[2] = -1;
  } else {
    mat iFIM= inv(FIM);

    cov[0] = iFIM(0,0);
    cov[1] = iFIM(1,1);
    cov[2] = iFIM(2,2);
  }
}
