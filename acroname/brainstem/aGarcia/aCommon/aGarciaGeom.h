/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGarciaGeom.h						   */
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

#ifndef _acpGarciaGeom_H_
#define _acpGarciaGeom_H_

#define aNUMGARCIAOUTLINEPOINTS		38

#define aNUMGARCIAGP2D12POINTS		4

#define	aGARCIA_RANGER_OFFSET_FRONT	4.38f
#define	aGARCIA_RANGER_OFFSET_SIDE	1.01f
#define	aGARCIA_RANGER_OFFSET_REAR	2.01f

#define aGARCIA_BATT_VOLTAGE_CONVFAC	102.4f
#define aGARCIA_BATT_CELLS_PER_PACK	6
#define aGARCIA_BATT_INTERP_POINTS	7


#ifdef __cplusplus
extern "C" {
#endif

extern float g_fBattV[aGARCIA_BATT_INTERP_POINTS];
extern float g_fBattM[aGARCIA_BATT_INTERP_POINTS];
extern float g_fBattB[aGARCIA_BATT_INTERP_POINTS];

extern float g_GarciaOutline[aNUMGARCIAOUTLINEPOINTS * 2];

extern float g_irfunc[aNUMGARCIAGP2D12POINTS][4];

typedef enum {
 kGarciaGeomNoRanger,
 kGarciaGeomFrontRanger,
 kGarciaGeomSideRanger,
 kGarciaGeomRearRanger,
 kGarciaGeomDownRanger
} eGarciaRangerType;


float aGarciaGeom_ScaleRangerData (
  const unsigned short rawValue,
  const eGarciaRangerType eType,
  const int nUnits
);

int aGarciaGeom_RangerToRaw (
  const float range,
  const eGarciaRangerType eType,
  const int nUnits
);

float aGarciaGeom_TicksToDistance (
  const long rawValue,
  const int nUnits
);

long aGarciaGeom_DistanceToTicks (
  const float fDistance,
  const int nUnits
);

short aGarciaGeom_SpeedToTicks(
  const float ups,
  const int nUnits
);

float aGarciaGeom_TicksToSpeed(
  const short ticks,
  const int nUnits
);

float aGarciaGeom_VoltageToCapacity (
  const float fVoltage
);

#ifdef __cplusplus 
}
#endif

#endif /* _acpGarciaGeom_H_ */
