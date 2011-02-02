/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aAnalog.c                                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: BrainStem analog-to-digital input routines.        */
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
#include "aAnalog.h"


#define aA2DREADTIMEOUTMS	250


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aAnalog_ReadInt
 */

aErr aAnalog_ReadInt(const aStemLib stemLib,
		     const unsigned char module,
		     const unsigned char nAnalogIndex,
		     int *pAnalogValue)
{
  aErr scErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  if ((stemLib == NULL) || (pAnalogValue == NULL))
    scErr = aErrParam;

  /* initialize the return value */
  if (scErr == aErrNone)
    *pAnalogValue = 0;

  /* send the outbound A/D request */
  if (scErr == aErrNone) {
    data[0] = cmdA2D_RD;
    data[1] = (char)(nAnalogIndex | maskDEV_VAL_HOST);
    if (!aPacket_Create(stemLib, module, 2, data, &packet, &scErr))
      aStem_SendPacket(stemLib, packet, &scErr);
  }

  /* seek the reply */
  if (scErr == aErrNone)
    aStem_GetPacket(stemLib,
		    aStemCore_CmdFilter,
		    (void*)cmdDEV_VAL,
      	            aA2DREADTIMEOUTMS,
      	            &packet,
      	            &scErr);

  if (scErr == aErrNone)
    aPacket_GetData(stemLib,
		    packet,
		    &address,
      		    &length,
      		    data,
      		    &scErr);
  
  if (scErr == aErrNone) {
      if (length == 4) {
        unsigned short uval;
        uval = aUtil_RetrieveUShort(&data[2]);
        *pAnalogValue = (int)(uval >> 6);
      } else
        scErr = aErrNotFound;
      aPacket_Destroy(stemLib, packet, &scErr);
  }

  return scErr;

} /* aAnalog_ReadInt */
