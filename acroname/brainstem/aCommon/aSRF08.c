/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSRF08.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of cross-platform SRF08 Ranger      */
/*              module interface routines.			   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/*        docs: Documentation can be found in the BrainStem        */
/*		reference under the major topic "C", category      */
/*              aSRF08                                             */
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
#include "aStemMsg.h"
#include "aUtil.h"
#include "aSRF08.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define aSRF08_GETDATATIMEOUTMS		100



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aBool sSRF08IICErrFilter(const unsigned char module,
			        const unsigned char dataLength,
			        const char* data,
			        void* ref);
static aBool sSRF08IICDataFilter(const unsigned char module,
			         const unsigned char dataLength,
			         const char* data,
			         void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSRF08IICErrFilter
 */

aBool sSRF08IICErrFilter(const unsigned char module,
			 const unsigned char dataLength,
		 	 const char* data,
			 void* ref)
{
  if ((dataLength > 1) 
      && (data[0] == (char)cmdMSG) 
      && (data[1] == iicNoAck))
    return aTrue;

  return aFalse;

} /* sSRF08IICErrFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSRF08IICDataFilter
 *
 * This uses the hard-coded device ID of 52 which is valid for 
 * the GP and Moto boards.  It should probably be more general.
 */

aBool sSRF08IICDataFilter(const unsigned char module,
			 const unsigned char dataLength,
		 	 const char* data,
			 void* ref)
{
  aAssert(data);

  if ((dataLength > 1) 
      && (data[0] == cmdDEV_VAL) 
      && (data[1] == 52))
    return aTrue;

  return aFalse;

} /* sSRF08IICErrFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSRF08_GetRange
 *
 * This routine initiates a sonar ranging from the addressed 
 * SRF08 and returns the requested number of range values
 * in the specified units.
 *
 * This routine requires the IIC baud rate to be set to 400kHz
 * as the SRF08 cannot handle 1MBit communciation.
 */

aErr aSRF08_GetRange(aStemLib stemLib,
		     const unsigned char router,
		     const unsigned char srf08address,
		     const unsigned char units,
		     short* pVals,
		     const unsigned char nVals)
{
  aErr srf08Err = aErrNone;
  unsigned char module;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  /* check all the parameters and get set up */
  if (!stemLib || !pVals || !router)
    srf08Err = aErrParam;
  if ((srf08Err == aErrNone)
      && ((units != aSRF08_INCH)
          && (units != aSRF08_CM)
          && (units != aSRF08_MS)))
    srf08Err = aErrParam;
  if ((srf08Err == aErrNone)
      && ((srf08address % 1)
          || (srf08address < 0xE0)
          || (srf08address == 0xFF)))
    srf08Err = aErrParam;   
  if ((srf08Err == aErrNone)
      && ((nVals == 0) || (nVals > aSRF08_NMAXREADINGS)))
    srf08Err = aErrRange;

  /* set the memory pointer in the SRF08 */
  if (srf08Err == aErrNone) {
    data[0] = 0;
    data[1] = (char)units;
    if (!aPacket_Create(stemLib, srf08address,
    		        2, data, &packet, &srf08Err))
      aStem_SendPacket(stemLib, packet, &srf08Err);    
  }

  /* now, wait for the reading and look for an IIC ACK error which 
   * would indicate no SRF08 connected at that address */ 
  if (srf08Err == aErrNone) {
    if (!aStem_GetPacket(stemLib, 
    		         sSRF08IICErrFilter,
    		         NULL,
    		         aSRF08_READDELAYMS,
    		         &packet,
    		         &srf08Err)) {
      aPacket_Destroy(stemLib, packet, NULL);
      srf08Err = aErrNotFound;
    } else {
      if (srf08Err == aErrTimeout)
        srf08Err = aErrNone;
    }
  }

  /* now, if there are no errors, we can assume there is:
   *  a) no brainstem attached at all
   *  b) a functioning SRF08 at the requested address
   *  c) some other IIC device at that address
   */

  /* set the SRF08 register pointer to the range data */
  if (srf08Err == aErrNone) {
    data[0] = 2;
    if (!aPacket_Create(stemLib, srf08address,
    		        1, data, &packet, &srf08Err))
      aStem_SendPacket(stemLib, packet, &srf08Err);    
  }

  /* read out the data up to 3 shorts at a time */
  if (srf08Err == aErrNone) {
    int i;
    short* p = pVals;
    int inc;
    for (i = nVals; (i > 0) && (srf08Err == aErrNone); i -= inc) {

      /* compute number of shorts for this pass */
      if (i > 3)
        inc = 3;
      else
        inc = i;

      /* request the next IIC read */
      data[0] = cmdIIC_RD;
      data[1] = (char)bit_IIC_RD_HOST;
      data[2] = (char)(srf08address + 1);
      data[3] = (char)(inc * 2);
      if (!aPacket_Create(stemLib, router,
    		          4, data, &packet, &srf08Err))
        aStem_SendPacket(stemLib, packet, &srf08Err);    

      /* now, seek the reply packet with the shorts */
      if (srf08Err == aErrNone) {
        if (!aStem_GetPacket(stemLib, 
    		             sSRF08IICDataFilter,
    		             NULL,
    		             aSRF08_GETDATATIMEOUTMS,
    		             &packet,
    		             &srf08Err)) {
          if (!aPacket_GetData(stemLib, packet, &module,
          		       &length, data, &srf08Err)) {
            if (length != (inc * 2 + 2))
              srf08Err = aErrIO;
            else {
              int j;
              char* v = &data[2];
              for (j = 0; j < inc; j++) {
                *p = aUtil_RetrieveShort(v);
                v += 2;
                p++;
              }
            }
          }
          aPacket_Destroy(stemLib, packet, NULL);

        } else {
          /* if we timed out, there is not likely a SRF08 present */
          if (srf08Err == aErrTimeout)
            srf08Err = aErrNotFound;
          
        }
      }
    }
  }

  return srf08Err;

} /* aSRF08_GetRange */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSRF08_GetLight
 *
 * This routine initiates a sonar ranging from the addressed 
 * SRF08 and returns light sensor value.
 *
 * This routine requires the IIC baud rate to be set to 400kHz
 * as the SRF08 cannot handle 1MBit communciation.
 */

aErr aSRF08_GetLight(aStemLib stemLib,
		     const unsigned char router,
		     const unsigned char srf08address,
		     unsigned char* pVal)
{
  aErr srf08Err = aErrNone;
  unsigned char module;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  /* check all the parameters and get set up */
  if (!stemLib || !pVal || !router)
    srf08Err = aErrParam;
  if ((srf08Err == aErrNone)
      && ((srf08address % 1)
          || (srf08address < 0xE0)
          || (srf08address == 0xFF)))
    srf08Err = aErrParam;   

  /* set the memory pointer in the SRF08 */
  if (srf08Err == aErrNone) {
    data[0] = 0;
    data[1] = (char)aSRF08_MS;
    if (!aPacket_Create(stemLib, srf08address,
    		        2, data, &packet, &srf08Err))
      aStem_SendPacket(stemLib, packet, &srf08Err);    
  }

  /* now, wait for the reading and look for an IIC ACK error which 
   * would indicate no SRF08 connected at that address */ 
  if (srf08Err == aErrNone) {
    if (!aStem_GetPacket(stemLib, 
    		         sSRF08IICErrFilter,
    		         NULL,
    		         aSRF08_READDELAYMS,
    		         &packet,
    		         &srf08Err)) {
      aPacket_Destroy(stemLib, packet, NULL);
      srf08Err = aErrNotFound;
    } else {
      if (srf08Err == aErrTimeout)
        srf08Err = aErrNone;
    }
  }

  /* now, if there are no errors, we can assume there is:
   *  a) no brainstem attached at all
   *  b) a functioning SRF08 at the requested address
   *  c) some other IIC device at that address
   */

  /* set the SRF08 register pointer to the light sensor data */
  if (srf08Err == aErrNone) {
    data[0] = 1;
    if (!aPacket_Create(stemLib, srf08address,
    		        1, data, &packet, &srf08Err))
      aStem_SendPacket(stemLib, packet, &srf08Err);    
  }

  /* request a read of the data */
  if (srf08Err == aErrNone) {
    data[0] = cmdIIC_RD;
    data[1] = (char)bit_IIC_RD_HOST;
    data[2] = (char)(srf08address + 1);
    data[3] = 1;
    if (!aPacket_Create(stemLib, router,
    		        4, data, &packet, &srf08Err))
      aStem_SendPacket(stemLib, packet, &srf08Err);
  }

  /* now, seek the reply packet with the light sensor value */
  if (srf08Err == aErrNone) {
    if (!aStem_GetPacket(stemLib, 
    		         sSRF08IICDataFilter,
    		         NULL,
    		         aSRF08_GETDATATIMEOUTMS,
    		         &packet,
    		         &srf08Err)) {
      if (!aPacket_GetData(stemLib, packet, &module,
          		   &length, data, &srf08Err)) {
        if (length != 3)
          srf08Err = aErrIO;
        else
          *pVal = (unsigned char)data[2];
      } else {
        /* if we timed out, there is not likely a SRF08 present */
        if (srf08Err == aErrTimeout)
          srf08Err = aErrNotFound;
          
      }
      aPacket_Destroy(stemLib, packet, NULL);
    }
  }

  return srf08Err;

} /* aSRF08_GetLight */
