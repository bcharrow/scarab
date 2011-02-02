/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGP2D02.c                                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: BrainStem GP2D02 ranger access routines.           */
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

#include "aGP2D02.h"
#include "aCmd.tea"


#define aGP2D02_TIMEOUT		200


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGP2D02_RawToInches
 *
 * returns GP2D02 measurement in inches from 3 - 18.5
 *
 */

double aGP2D02_RawToInches(unsigned char raw)
{
  double val = 0;
  if (raw > 150)
    val = 6.0 - ((double)raw - 150.0) / 20.0;
  else if (raw > 115)
    val = 10.0 - ((double)raw - 115.0) / 10.0;
  else if (raw > 89)
    val = 18.5 - ((double)raw - 89.0) / 2.8;

  return val;

} /* aGP2D02_RawToInches */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGP2D02_ReadInt
 */

aErr aGP2D02_ReadInt(const aStemLib stemRef,
		     const unsigned char module,
		     const unsigned char rangerIndex,
		     int* pRangerValue)
{
  aErr irErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char address;
  unsigned char length;
  aPacketRef packet;

  if ((stemRef == NULL) || (pRangerValue == NULL))
    irErr = aErrParam;

  /* send the ranger read request */
  if (irErr == aErrNone) {
    data[0] = cmdIR02_RD;
    data[1] = (char)(rangerIndex | maskDEV_VAL_HOST);
    if (!aPacket_Create(stemRef, 
    			module, 2, 
    			data,
    			&packet, 
    			&irErr))
      aStem_SendPacket(stemRef, packet, &irErr);
  }

  /* seek the reply */
  if (irErr == aErrNone) {
    if (!aStem_GetPacket(stemRef, 
    			 aStemCore_CmdFilter, 
    			 (void*)cmdDEV_VAL,
      		         aGP2D02_TIMEOUT, 
      		         &packet, 
      		         &irErr)
        && !aPacket_GetData(stemRef, 
        		    packet, 
        		    &address,
      		            &length, 
      		            data, 
      		            &irErr)) {
      if (length == 3) {
        unsigned char uval = (unsigned char)data[2];
        *pRangerValue = uval;
      } else
        irErr = aErrNotFound;
      aPacket_Destroy(stemRef, packet, &irErr);
    }
  }

  return irErr;

} /* aGP2D02_ReadInt */
