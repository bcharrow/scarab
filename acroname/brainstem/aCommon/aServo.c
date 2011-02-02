/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aServo.c     	                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of servo control routines for the   */
/*		BrainStem modules.	    			   */
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

#include "aCmd.tea"

#include "aUtil.h"
#include "aServo.h"

#define aSERVOREPLYTIMEOUTMS	100


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_GetPosition
 */

aErr aServo_GetPosition(aStemLib stemLib,
			const unsigned char module,
			const unsigned char nServoIndex,
			unsigned char *pServoPosition)
{
  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  aAssert(stemLib);
  aAssert(pServoPosition);

  data[0] = cmdSRV_ABS;
  data[1] = (char)nServoIndex;

  /* build and send the packet */
  aPacket_Create(stemLib, module, 2, data, &packet, &servoErr);
  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  /* look for the reply */
  if (servoErr == aErrNone)
    aStem_GetPacket(stemLib, aStemCore_CmdFilter,
                    (void*)cmdSRV_ABS, 
      		    aSERVOREPLYTIMEOUTMS,
      		    &packet,
      		    &servoErr);

  /* get the packet data */
  if (servoErr == aErrNone)
    aPacket_GetData(stemLib, packet, &address, &length, data, &servoErr);

  if (servoErr == aErrNone) {
    *pServoPosition = (unsigned char)data[2];
    aPacket_Destroy(stemLib, packet, &servoErr);
  }

  return servoErr;

} /* aServo_GetPosition */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_GetServoConfig
 */

aErr aServo_GetConfig(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				unsigned char *pConfigByte)
{
  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  aAssert(stemLib);
  aAssert(pConfigByte);  

  data[0] = cmdSRV_CFG;
  data[1] = (char)nServoIndex;

  /* build and send the packet */
  aPacket_Create(stemLib, module, 2, data, &packet, &servoErr);
  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  /* look for the reply */
  if (servoErr == aErrNone)
    aStem_GetPacket(stemLib, aStemCore_CmdFilter, (void*)cmdSRV_CFG, 
      		    aSERVOREPLYTIMEOUTMS, &packet, &servoErr);

  /* get the packet data */
  if (servoErr == aErrNone)
    aPacket_GetData(stemLib, packet, &address, &length, data, &servoErr);

  if (servoErr == aErrNone) {
    *pConfigByte = (unsigned char)data[2];
    aPacket_Destroy(stemLib, packet, &servoErr);
  }

  return servoErr;

} /* aServo_GetServoConfig */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_GetLimits
 */

aErr aServo_GetLimits(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				unsigned char *pLimitBytes)
{				

  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  aAssert(stemLib);
  aAssert(pLimitBytes);  

  data[0] = cmdSRV_LMT;
  data[1] = (char)nServoIndex;

  /* build and send the packet */
  aPacket_Create(stemLib, module, 2, data, &packet, &servoErr);
  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  /* look for the reply */
  if (servoErr == aErrNone)
    aStem_GetPacket(stemLib,
                    aStemCore_CmdFilter,
                    (void*)cmdSRV_LMT, 
      		    aSERVOREPLYTIMEOUTMS,
      		    &packet,
      		    &servoErr);

  /* get the packet data */
  if (servoErr == aErrNone)
    aPacket_GetData(stemLib, packet, &address, &length, data, &servoErr);

  if (servoErr == aErrNone) {
    aMemCopy(pLimitBytes, &data[2], 2);
    aPacket_Destroy(stemLib, packet, &servoErr);
  }

  return servoErr;

} /* aServo_GetServoLimits */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_SetPositionAbs
 */

aErr aServo_SetPositionAbs(aStemLib stemLib,
				     const unsigned char module,
				     const unsigned char nServoIndex,
				     const unsigned char nPosition)
{
  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  aAssert(stemLib);

  data[0] = cmdSRV_ABS;
  data[1] = (char)nServoIndex;
  data[2] = (char)nPosition;

  aPacket_Create(stemLib, module, 3, data, &packet, &servoErr);

  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  return servoErr;

} /* aServo_SetServoPositionAbs */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_SetServoPositionRel
 */

aErr aServo_SetPositionRel(aStemLib stemLib,
				     const unsigned char module,
				     const unsigned char nServoIndex,
				     const int nChange)
{
  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  char cSign = 0;
  int nFix = nChange;

  aAssert(stemLib);

  if (nFix < 0) {
    cSign = 1;
    nFix = -nFix;
  }
  if (nFix > 255) nFix = 255;  

  data[0] = cmdSRV_REL;
  data[1] = (char)nServoIndex;
  data[2] = (char)nFix;
  data[3] = cSign;

  aPacket_Create(stemLib, module, 4, data, &packet, &servoErr);

  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  return servoErr;

} /* aServo_SetServoPositionRel */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_SetConfig
 */

aErr aServo_SetConfig(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				const unsigned char configByte)
{
  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  aAssert(stemLib);

  data[0] = cmdSRV_CFG;
  data[1] = (char)nServoIndex;
  data[2] = (char)configByte;

  aPacket_Create(stemLib, module, 3, data, &packet, &servoErr);

  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  return servoErr;

} /* aServo_SetServoConfig */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_SetServoLimits
 */

aErr aServo_SetLimits(aStemLib stemLib,
				const unsigned char module,
				const unsigned char nServoIndex,
				const unsigned char *pLimitBytes)
{
  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  aAssert(stemLib);
  aAssert(pLimitBytes);  

  data[0] = cmdSRV_LMT;
  data[1] = (char)nServoIndex;
  aMemCopy(&data[2], pLimitBytes, 2);

  /* build and send the packet */
  aPacket_Create(stemLib, module, 4, data, &packet, &servoErr);
  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  return servoErr;

} /* aServo_SetLimits */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aServo_CommitSettings
 */

aErr aServo_CommitSettings(aStemLib stemLib,
			   const unsigned char module)
{

  aErr servoErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  aAssert(stemLib);

  data[0] = cmdSRV_SAV;

  /* build and send the packet */
  aPacket_Create(stemLib, module, 1, data, &packet, &servoErr);
  if (servoErr == aErrNone)
    aStem_SendPacket(stemLib, packet, &servoErr);

  return servoErr;

} /* aServo_CommitSettings */
