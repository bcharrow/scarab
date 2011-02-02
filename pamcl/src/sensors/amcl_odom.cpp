/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL odometry routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_odom.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <algorithm>

#include <sys/types.h> // required by Darwin
#include <math.h>

#include "amcl_odom.h"

using namespace amcl;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLOdom::AMCLOdom() : AMCLSensor()
{
  this->time = 0.0;
  lx = ly = la = 0;
  prev_lx = prev_ly = prev_la = 0;
  lcov_x = lcov_y = lcov_a = 0;
}

void
AMCLOdom::SetModelDiff(double alpha1, 
                       double alpha2, 
                       double alpha3, 
                       double alpha4)
{
  this->model_type = ODOM_MODEL_DIFF;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
}

void
AMCLOdom::SetModelOmni(double alpha1, 
                       double alpha2, 
                       double alpha3, 
                       double alpha4,
                       double alpha5)
{
  this->model_type = ODOM_MODEL_OMNI;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
  this->alpha5 = alpha5;
}

void 
AMCLOdom::SetLodom(double lx, double ly, double la, 
			double lcov_x, double lcov_y, double lcov_a)
{
  this->lx = lx;
  this->ly = ly;
  this->la = la;
  this->lcov_x = lcov_x;
  this->lcov_y = lcov_y;
  this->lcov_a = lcov_a;
} 


////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool AMCLOdom::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
  //std::cout <<"Cov: "<<lcov_x<<std::endl;
  AMCLOdomData *ndata;
  ndata = (AMCLOdomData*) data;

  // Compute the new sample poses
  pf_sample_set_t *set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(ndata->pose, ndata->delta);

  if (this->model_type == ODOM_MODEL_DIFF)	//DIFF is the Only supporting model now
  {
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    //These are for LASER ODOMETRY==============================================================
    double ldelta_rot1, ldelta_trans, ldelta_rot2;
    double ldelta_rot1_noise, ldelta_rot2_noise;
    double ldelta_x = lx - prev_lx;
    double ldelta_y = ly - prev_ly;
    double ldelta_a = la - prev_la;

    ldelta_rot1 = angle_diff(atan2(ldelta_y, ldelta_x), prev_la);
    ldelta_trans = sqrt(ldelta_x*ldelta_x + ldelta_y*ldelta_y);
    ldelta_rot2 = angle_diff(ldelta_a, ldelta_rot1);

    ldelta_rot1_noise = std::min(fabs(angle_diff(ldelta_rot1,0.0)),
                                 fabs(angle_diff(ldelta_rot1,M_PI)));
    ldelta_rot2_noise = std::min(fabs(angle_diff(ldelta_rot2,0.0)),
                                 fabs(angle_diff(ldelta_rot2,M_PI)));
    prev_lx = lx;
    prev_ly = ly;
    prev_la = la;
    //These are for LASER ODOMETRY==============================================================

    //These are for WHEEL ODOMETRY==============================================================
    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_noise, delta_rot2_noise;

    if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] + 
            ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
      delta_rot1 = 0.0;
    else
      delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                              old_pose.v[2]);
    delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                       ndata->delta.v[1]*ndata->delta.v[1]);
    delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);

    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));
    //These are for WHEEL ODOMETRY==============================================================

    for (int i = 0; i < set->sample_count; i++)
    {
      int rnd = rand() % 2;
      int is_laser = (rnd == 0)?1:0;
      pf_sample_t* sample = set->samples + i;

      if (is_laser)
      {
        delta_rot1_hat = angle_diff(ldelta_rot1, pf_ran_gaussian(lcov_a));

        delta_trans_hat = ldelta_trans - pf_ran_gaussian(lcov_x);;

        delta_rot2_hat = angle_diff(ldelta_rot2, pf_ran_gaussian(lcov_a));
      }
      else
      {
        // Sample pose differences
        delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                  this->alpha2*delta_trans*delta_trans));

        delta_trans_hat = delta_trans - 
              pf_ran_gaussian(this->alpha3*delta_trans*delta_trans +
                              this->alpha4*delta_rot1_noise*delta_rot1_noise +
                              this->alpha4*delta_rot2_noise*delta_rot2_noise);
        delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                  this->alpha2*delta_trans*delta_trans));
      }

      // Apply sampled update to particle pose
      sample->pose.v[0] += delta_trans_hat * 
              cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat * 
              sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
      sample->weight = 1.0 / set->sample_count;
    }
  }

  return true;
}
