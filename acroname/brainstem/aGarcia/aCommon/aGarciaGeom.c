/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGarciaGeom.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Geometry information for Garcia consolodated into  */
/*              a single file.                  		   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2008. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "aGarciaDefs.tea"
#include "aGarciaGeom.h"

#define aGARCIA_PID_TIME_CONSTANT	((float)aGARCIA_PID_PERIOD / 10000.0f)

float g_fBattV[aGARCIA_BATT_INTERP_POINTS] = { 1.38f, 1.32f, 1.3f, 1.28f, 1.24f, 1.22f, 1.15f };
float g_fBattM[aGARCIA_BATT_INTERP_POINTS] = { 0.0f, 1.166667f, 3.5f, 14.0f, 7.0f, 3.5f, 2.0f };
float g_fBattB[aGARCIA_BATT_INTERP_POINTS] = { 1.0f, -0.61f, -3.69f, -17.34f, -8.38f, -4.04f, -2.21f };

float g_GarciaOutline[aNUMGARCIAOUTLINEPOINTS * 2] = {
  -3.75f,  2.10f,
  -2.50f,  3.50f,
  -0.90f,  4.20f,
   0.90f,  4.20f,
   2.50f,  3.50f,
   3.75f,  2.10f,
   3.10f,  2.10f,
   3.10f,  0.125f,
   3.20f,  0.125f,
   3.20f,  1.90f,

   3.30f,  2.00f,
   3.70f,  2.00f,
   3.80f,  1.90f,
   3.80f, -1.90f,
   3.70f, -2.00f,
   3.30f, -2.00f,
   3.20f, -1.90f,
   3.20f, -0.125f,
   3.10f, -0.125f,
   3.10f, -2.10f,

   2.50f, -2.10f,
   1.10f, -6.50f,
  -1.10f, -6.50f,
  -2.50f, -2.10f,
  -3.10f, -2.10f,
  -3.10f, -0.125f,
  -3.20f, -0.125f,
  -3.20f, -1.90f,
  -3.30f, -2.00f,
  -3.70f, -2.00f,

  -3.80f, -1.90f,
  -3.80f,  1.90f,
  -3.70f,  2.00f,
  -3.30f,  2.00f,
  -3.20f,  1.90f,
  -3.20f,  0.125f,
  -3.10f,  0.125f,
  -3.10f,  2.10f 
};


/* inches, points, raw value, scale factor */
float g_irfunc[aNUMGARCIAGP2D12POINTS][4] = {
  {7.0f, 3.0f, 321.0f, 185.0f},
  {11.0f, 4.0f, 220.0f, 101.0f},
  {15.0f, 4.0f, 181.0f, 39.0f},
  {18.0f, 3.0f, 176.0f, 5.0f}
};



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGarciaGeom_ScaleRangerData
 */

float aGarciaGeom_ScaleRangerData(
  const unsigned short rawValue,
  const eGarciaRangerType eType,
  const int nUnits)
{
  int v = rawValue;
  float r = 0.0f;
  float rOffset = 0.0f;
  int i;

  /* interpolate 10-bit raw voltage to inches */
  r = 0.0f;
  for (i = 0; i < aNUMGARCIAGP2D12POINTS; i++) {
    if (v > g_irfunc[i][2]) {
      r =   g_irfunc[i][0]
          - g_irfunc[i][1] * (v - g_irfunc[i][2]) / g_irfunc[i][3];
      break;
    }
  }
  
  switch (eType)
  {
    case kGarciaGeomFrontRanger:
      rOffset = aGARCIA_RANGER_OFFSET_FRONT; break;
    case kGarciaGeomSideRanger:
      rOffset = aGARCIA_RANGER_OFFSET_SIDE; break;
    case kGarciaGeomRearRanger:
      rOffset = aGARCIA_RANGER_OFFSET_REAR; break;
    case kGarciaGeomDownRanger:
    default:
      rOffset = 0.0f; break;
  }
  
  /* add centering offset in inches if result is valid */
  if (r > 0.0f)
    r += rOffset;
  
  /* convert inches to whatever units we're using */
  switch (nUnits) {
    default:
    case aGARCIA_DISTANCE_METERS:
      /* inches to meters */
      r /= 39.37008f;
      break;
    case aGARCIA_DISTANCE_FEET:
      /* inches to feet */
      r /= 12.0f;
      break;
    case aGARCIA_DISTANCE_INCHES:
      /* inches to inches */
      break;
  }
  return r;

} /* aGarciaGeom_ScaleRangerData */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGarciaGeom_RangerToRaw
 */

int aGarciaGeom_RangerToRaw(
  const float range,
  const eGarciaRangerType eType,
  const int nUnits)
{
  int v = 0;
  float r;
  float rOffset = 0.0f;
  int i;
  
  r = range;

  /* convert whatever we're using to inches */
  switch (nUnits) {
    default:
    case aGARCIA_DISTANCE_METERS:
      /* meters to inches */
      r *= 39.37008f;
      break;
    case aGARCIA_DISTANCE_FEET:
      /* feet to inches */
      r *= 12.0f;
      break;
    case aGARCIA_DISTANCE_INCHES:
      /* inches to inches */
      break;
  }

  switch (eType)
  {
    case kGarciaGeomFrontRanger:
      rOffset = aGARCIA_RANGER_OFFSET_FRONT; break;
    case kGarciaGeomSideRanger:
      rOffset = aGARCIA_RANGER_OFFSET_SIDE; break;
    case kGarciaGeomRearRanger:
      rOffset = aGARCIA_RANGER_OFFSET_REAR; break;
    case kGarciaGeomDownRanger:
    default:
      rOffset = 0.0f; break;
  }
  
  /* subtract centering offset in inches if result is valid */
  if (r > 0.0f)
    r -= rOffset;

  /* interpolate inches to 10-bit raw voltage */
  v = 0;
  for (i = 0; i < aNUMGARCIAGP2D12POINTS; i++) {
    if (r < g_irfunc[i][0]) {
      v = (int)((((r - g_irfunc[i][0]) * g_irfunc[i][3]) / -g_irfunc[i][1])
                + g_irfunc[i][2]);
      break;
    }
  }
  
  return v;

} /* aGarciaGeom_RangerToRaw */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGarciaGeom_TicksToDistance
 */

float aGarciaGeom_TicksToDistance(
  const long rawValue,
  const int nUnits)
{
  float r = 0;
  switch(nUnits)
  {
    case aGARCIA_DISTANCE_METERS:
      r = ((float)rawValue) / aGARCIA_TICKS_PER_METER;
      break;
    case aGARCIA_DISTANCE_FEET:
      r = ((float)rawValue) / aGARCIA_TICKS_PER_FOOT;
      break;
    case aGARCIA_DISTANCE_INCHES:
      r = ((float)rawValue) / (aGARCIA_TICKS_PER_FOOT / 12.0f);
      break;
  }
  return r;

} /* aGarciaGeom_TicksToDistance */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGarciaGeom_SpeedToTicks
 */

short aGarciaGeom_SpeedToTicks(
  const float ups,
  const int nUnits)
{
  short v = 0;
  switch(nUnits)
  {
    case aGARCIA_DISTANCE_METERS:
      v = (short)(ups * aGARCIA_PID_TIME_CONSTANT * (float)aGARCIA_TICKS_PER_METER);
      break;
    case aGARCIA_DISTANCE_FEET:
      v = (short)(ups * aGARCIA_PID_TIME_CONSTANT * (float)aGARCIA_TICKS_PER_FOOT);
      break;
    case aGARCIA_DISTANCE_INCHES:
      v = (short)(ups * aGARCIA_PID_TIME_CONSTANT * (float)(aGARCIA_TICKS_PER_FOOT / 12.0f));
      break;
  }
  if (v > aGARCIA_MAX_VELOCITY) v = aGARCIA_MAX_VELOCITY;
  if (v < -aGARCIA_MAX_VELOCITY) v = -aGARCIA_MAX_VELOCITY;
  return v;

} /* aGarciaGeom_SpeedToTicks */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGarciaGeom_TicksToSpeed
 */

float aGarciaGeom_TicksToSpeed(
  const short ticks,
  const int nUnits)
{
  float v = 0;
  switch(nUnits)
  {
    case aGARCIA_DISTANCE_METERS:
      v = ticks / (aGARCIA_PID_TIME_CONSTANT * (float)aGARCIA_TICKS_PER_METER);
      break;
    case aGARCIA_DISTANCE_FEET:
      v = ticks / (aGARCIA_PID_TIME_CONSTANT * (float)aGARCIA_TICKS_PER_FOOT);
      break;
    case aGARCIA_DISTANCE_INCHES:
      v = ticks / (aGARCIA_PID_TIME_CONSTANT * (float)(aGARCIA_TICKS_PER_FOOT / 12.0f));
      break;
  }
  if (v > aGARCIA_MAX_VELOCITY) v = aGARCIA_MAX_VELOCITY;
  if (v < -aGARCIA_MAX_VELOCITY) v = -aGARCIA_MAX_VELOCITY;
  return v;

} /* aGarciaGeom_TicksToSpeed */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGarciaGeom_DistanceToTicks
 */

long aGarciaGeom_DistanceToTicks(
  const float fDistance,
  const int nUnits)
{
  long n = 0;
  switch (nUnits)
  {
    case aGARCIA_DISTANCE_METERS:
      n = (long)(fDistance * aGARCIA_TICKS_PER_METER);
      break;
    case aGARCIA_DISTANCE_FEET:
      n = (long)(fDistance * aGARCIA_TICKS_PER_FOOT);
      break;
    case aGARCIA_DISTANCE_INCHES:
      n = (long)(fDistance * (aGARCIA_TICKS_PER_FOOT / 12.0f));
      break;
  }
  return n;

} /* aGarciaGeom_DistanceToTicks */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGarciaGeom_VoltageToCapacity
 *
 * Returns the normalized battery capacity left (0.0 - 1.0)
 * based on a "standard" 0.2C discharge curve.
*/

float aGarciaGeom_VoltageToCapacity (
  const float fVoltage)
{
  float fRetVal;
  float vadj;
  int i;

  /* interpolation is for 1 cell */
  vadj = fVoltage / (float)aGARCIA_BATT_CELLS_PER_PACK;

  fRetVal = 0.0f;
  for (i = 0; i < aGARCIA_BATT_INTERP_POINTS; i++) {
    if (vadj > g_fBattV[i]) {
      fRetVal = g_fBattM[i] * vadj + g_fBattB[i];
      break;
    }
  }
  
  return fRetVal;

} /* aGarciaGeom_VoltageToCapacity */

