/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aIR.c                                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: BrainStem GP 2.0 IR routines.                      */
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
#include "aIR.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define aIR_GETDATATIMEOUTMS  250	
#define aIR_RXPIN 4
#define aIR_TXPIN 5		 

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIR_ConfigRX
 */
aErr aIR_ConfigRX(const aStemLib stemRef,
		  const unsigned char module,
		  char irConfiguration)
{
  aErr irErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  
  /* Check to make sure a stem library reference has been passed */
  if (stemRef == NULL)
    irErr = aErrParam;
  
  /* send the digital config command */
  if (irErr == aErrNone) {
    data[0] = cmdDIG_CFG;
    data[1] = (char) aIR_RXPIN;
    data[2] = (char)(aIR_RXMASK | maskDEV_VAL_HOST);
    if (!aPacket_Create(stemRef, 
    			module, 3, 
    			data,
    			&packet, 
    			&irErr))
      aStem_SendPacket(stemRef, packet, &irErr);
  }

  /* send the digital ptime command */
  if (irErr == aErrNone) {
    data[0] = cmdPTIME_RD;
    data[1] = (char)(aIR_RXPIN | maskDEV_VAL_HOST);
    if (!aPacket_Create(stemRef, 
    			module, 2, 
    			data,
    			&packet, 
    			&irErr))
      aStem_SendPacket(stemRef, packet, &irErr);
  }  
  return irErr;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIR_ConfigTX
 */

aErr aIR_ConfigTX(const aStemLib stemRef,
		  const unsigned char module,
		  char irConfiguration)
{
  aErr irErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  
  /* Check to make sure a stem library reference has been passed */
  if (stemRef == NULL)
    irErr = aErrParam;
  
  /* send the digital config command */
  if (irErr == aErrNone) {
    data[0] = cmdDIG_CFG;
    data[1] = (char)(aIR_TXPIN);
    data[2] = (char)aIR_TXMASK;
    if (!aPacket_Create(stemRef, 
    			module, 3, 
    			data,
    			&packet, 
    			&irErr))
      aStem_SendPacket(stemRef, packet, &irErr);
  }
  
  return irErr;
  
} /* aIR_TXInt */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIR_TXInt
 */

aErr aIR_TXInt(const aStemLib stemRef,
	       const unsigned char module,
	       int irValue)
{
  aErr irErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  
  /* Check to make sure a stem library reference has been passed */
  if (stemRef == NULL)
    irErr = aErrParam;
  
  /* send the digital config command with IR settings */
  if (irErr == aErrNone) {
    data[0] = cmdIRP_XMIT;
    data[1] = (char)(aIR_TXPIN);
    data[2] = (char)((irValue & 0xFF00) >> 8);
    data[3] = (char)(irValue & 0x00FF);	
    if (!aPacket_Create(stemRef, 
    			module, 4, 
    			data,
    			&packet, 
    			&irErr))
      aStem_SendPacket(stemRef, packet, &irErr);
  }
  
  return irErr;
  
} /* aIR_TXInt */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIR_RXInt
 */

aErr aIR_RXInt(const aStemLib stemRef,
	       const unsigned char module,
	       int* pValue)
{
  aErr irErr = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char length;
  unsigned char address;
  aPacketRef packet;
  
  if ((stemRef == NULL) || (pValue == NULL))
    irErr = aErrParam;

  /* seek the reply */
  if (irErr == aErrNone) {
    if (!aStem_GetPacket(stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdDEV_VAL,
      		         aIR_GETDATATIMEOUTMS, 
      		         &packet, 
      		         &irErr)
        && !aPacket_GetData(stemRef, 
        		    packet, 
        		    &address,
      		            &length, 
      		            data, 
      		            &irErr)) {
      if (length == 4) {
      
        unsigned short uval;
        uval = aUtil_RetrieveUShort(&data[2]);
        *pValue = uval;
      } else
        irErr = aErrNotFound;
	
      aPacket_Destroy(stemRef, packet, &irErr);
    }
  }

  return irErr;
  } /* aIR_TXInt */

