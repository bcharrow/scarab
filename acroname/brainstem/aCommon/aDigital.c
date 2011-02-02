/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aDigital.c                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: BrainStem digital IO routines.                     */
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

#include "aUtil.h"
#include "aCmd.tea"
#include "aDigital.h"


#define aDIGITAL_TIMEOUT	250


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDigital_ReadInt
 */

aErr aDigital_ReadInt(const aStemLib stemRef,
		     const unsigned char module,
		     const unsigned char nDigitalIndex,
		     int* pDigitalValue)
{
  aErr digErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  if ((stemRef == NULL) || (pDigitalValue == NULL))
    digErr = aErrParam;

  /* send the digital input request */
  if (digErr == aErrNone) {
    data[0] = cmdDIG_IO;
    data[1] = (char)(nDigitalIndex | maskDEV_VAL_HOST);
    if (!aPacket_Create(stemRef, 
    			module, 2, 
    			data,
    			&packet, 
    			&digErr))
      aStem_SendPacket(stemRef, packet, &digErr);
  }

  /* seek the reply */
  if (digErr == aErrNone) {
    if (!aStem_GetPacket(stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdDEV_VAL,
      		         aDIGITAL_TIMEOUT, 
      		         &packet, 
      		         &digErr)
        && !aPacket_GetData(stemRef, 
        		    packet, 
        		    &address,
      		            &length, 
      		            data, 
      		            &digErr)) {
      if (length == 3) {
        unsigned char uval = (unsigned char)data[2];
        *pDigitalValue = uval;
      } else
        digErr = aErrNotFound;
      aPacket_Destroy(stemRef, packet, &digErr);
    }
  }

  return digErr;

} /* aDigital_ReadInt */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDigital_ReadTmr
 */

aErr aDigital_ReadTmr(const aStemLib stemRef,
		     const unsigned char module,
		     const unsigned char nDigitalIndex,
		     int* pTimerValue)
{
  aErr digErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  if ((stemRef == NULL) || (pTimerValue == NULL))
    digErr = aErrParam;

  /* send the digital timer input request */
  if (digErr == aErrNone) {
    data[0] = cmdPTIME_RD;
    data[1] = (char)(nDigitalIndex | maskDEV_VAL_HOST);
    if (!aPacket_Create(stemRef, 
    			module, 2, 
    			data,
    			&packet, 
    			&digErr))
      aStem_SendPacket(stemRef, packet, &digErr);
  }

  /* seek the reply */
  if (digErr == aErrNone) {
    if (!aStem_GetPacket(stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdDEV_VAL,
      		         aDIGITAL_TIMEOUT, 
      		         &packet, 
      		         &digErr)
        && !aPacket_GetData(stemRef, 
        		    packet, 
        		    &address,
      		            &length, 
      		            data, 
      		            &digErr)) {
      if (length == 4) {
        unsigned short uval;
        uval = aUtil_RetrieveUShort(&data[2]);
        *pTimerValue = (int)(uval);
      } else
        digErr = aErrNotFound;
      aPacket_Destroy(stemRef, packet, &digErr);
    }
  }

  return digErr;

} /* aDigital_ReadInt */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDigital_GetConfig
 */

aErr aDigital_GetConfig(const aStemLib stemRef,
		     const unsigned char module,
		     const unsigned char nDigitalIndex,
		     int* pConfigValue)
{
  aErr digErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  if ((stemRef == NULL) || (pConfigValue == NULL))
    digErr = aErrParam;

  /* send the digital config read request */
  if (digErr == aErrNone) {
    data[0] = cmdDIG_CFG;
    data[1] = (char)(nDigitalIndex);
    if (!aPacket_Create(stemRef, 
    			module, 2, 
    			data,
    			&packet, 
    			&digErr))
      aStem_SendPacket(stemRef, packet, &digErr);
  }

  /* seek the reply */
  if (digErr == aErrNone) {
    if (!aStem_GetPacket(stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdDIG_CFG,
      		         aDIGITAL_TIMEOUT, 
      		         &packet, 
      		         &digErr)
        && !aPacket_GetData(stemRef, 
        		    packet, 
        		    &address,
      		            &length, 
      		            data, 
      		            &digErr)) {
      if (length == 3) {
        unsigned char uval = (unsigned char)data[2];
        *pConfigValue = uval;
      } else
        digErr = aErrNotFound;
      aPacket_Destroy(stemRef, packet, &digErr);
    }
  }

  return digErr;

} /* aDigital_GetConfig */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDigital_WriteInt
 */

aErr aDigital_WriteInt(const aStemLib stemRef,
		       const unsigned char module,
		       const unsigned char nDigitalIndex,
		       int nDigitalValue)
{
  aErr digErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  if (stemRef == NULL)
    digErr = aErrParam;

  /* send the digital output command */
  if (digErr == aErrNone) {
    data[0] = cmdDIG_IO;
    data[1] = (char)(nDigitalIndex);
    data[2] = (char)((nDigitalValue > 0) ? 1 : 0);
    if (!aPacket_Create(stemRef, 
    			module, 3, 
    			data,
    			&packet, 
    			&digErr))
      aStem_SendPacket(stemRef, packet, &digErr);
  }

  return digErr;

} /* aDigital_WriteInt */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDigital_SetConfig
 */

aErr aDigital_SetConfig(const aStemLib stemRef,
		       const unsigned char module,
		       const unsigned char nDigitalIndex,
		       int nConfigValue)
{
  aErr digErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  if (stemRef == NULL)
    digErr = aErrParam;

  /* send the digital config command */
  if (digErr == aErrNone) {
    data[0] = cmdDIG_CFG;
    data[1] = (char)(nDigitalIndex);
    data[2] = (char)(nConfigValue);
    if (!aPacket_Create(stemRef, 
    			module, 3, 
    			data,
    			&packet, 
    			&digErr))
      aStem_SendPacket(stemRef, packet, &digErr);
  }

  return digErr;

} /* aDigital_SetConfig */
