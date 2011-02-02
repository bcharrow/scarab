/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aCMPS03.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of cross-platform CMPS03 Compass    */
/*              module interface routines.			   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/*        docs: Documentation can be found in the BrainStem        */
/*		reference under the major topic "C", category      */
/*              aCMPS03                                            */
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
#include "aCMPS03.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local defines
 */

#define aCMPS03_ADDRESS			0xC0
#define aCMPS03_GETDATATIMEOUTMS	100



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aBool sCMPS03IICErrFilter(const unsigned char module,
			        const unsigned char dataLength,
			        const char* data,
			        void* ref);
static aBool sCMPS03IICDataFilter(const unsigned char module,
			         const unsigned char dataLength,
			         const char* data,
			         void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCMPS03IICErrFilter
 */

aBool sCMPS03IICErrFilter(const unsigned char module,
			 const unsigned char dataLength,
		 	 const char* data,
			 void* ref)
{
  if ((dataLength > 1) 
      && (data[0] == cmdMSG) 
      && (data[1] == iicNoAck))
    return aTrue;

  return aFalse;

} /* sCMPS03IICErrFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCMPS03IICDataFilter
 *
 * This uses the hard-coded device ID of 52 which is valid for 
 * the GP and Moto boards.  It should probably be more general.
 */

aBool sCMPS03IICDataFilter(const unsigned char module,
			 const unsigned char dataLength,
		 	 const char* data,
			 void* ref)
{
  aAssert(pAddr);

  if ((dataLength > 1) 
      && (data[0] == cmdDEV_VAL) 
      && (data[1] == 52))
    return aTrue;

  return aFalse;

} /* sCMPS03IICDataFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aCMPS03_GetHeading
 *
 * This routine retrieves the compass heading and returns the
 * heading in the units requested.
 *
 * This routine requires the IIC baud rate to be set to 100kHz
 * as the CMPS03 cannot handle 1MBit or 400kHz communciation.
 */

aErr aCMPS03_GetHeading(aStemLib stemLib,
		        const unsigned char router,
		        const unsigned char units,
		        float* pVal)
{
  aErr cmpsErr = aErrNone;
  unsigned char module;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  /* check all the parameters and get set up */
  if (!stemLib || !pVal || !router)
    cmpsErr = aErrParam;
  if ((cmpsErr == aErrNone)
      && ((units != aCMPS03_RADIANS)
          && (units != aCMPS03_DEGREES)
          && (units != aCMPS03_ROTATIONS)))
    cmpsErr = aErrParam;

  /* set the memory pointer in the CMPS03 */
  if (cmpsErr == aErrNone) {
    data[0] = 2;
    if (!aPacket_Create(stemLib, aCMPS03_ADDRESS,
    		        1, data, &packet, &cmpsErr))
      aStem_SendPacket(stemLib, packet, &cmpsErr);    
  }

  /* now, wait for a bit and look for an IIC ACK error which 
   * would indicate no CMPS03 connected at that address */ 
  if (cmpsErr == aErrNone) {
    if (!aStem_GetPacket(stemLib, 
    		         sCMPS03IICErrFilter,
    		         NULL,
    		         50,
    		         &packet,
    		         &cmpsErr)) {
      aPacket_Destroy(stemLib, packet, NULL);
      cmpsErr = aErrNotFound;
    } else {
      if (cmpsErr == aErrTimeout)
        cmpsErr = aErrNone;
    }
  }

  /* now, if there are no errors, we can assume there is:
   *  a) no brainstem attached at all
   *  b) a functioning CMPS03 at the requested address
   *  c) some other IIC device at that address
   */

  /* request a read of the data */
  if (cmpsErr == aErrNone) {
    data[0] = cmdIIC_RD;
    data[1] = (char)bit_IIC_RD_HOST;
    data[2] = (char)(aCMPS03_ADDRESS + 1);
    data[3] = 2;
    if (!aPacket_Create(stemLib, router,
    		        4, data, &packet, &cmpsErr))
      aStem_SendPacket(stemLib, packet, &cmpsErr);
  }

  /* now, seek the reply packet with the light sensor value */
  if (cmpsErr == aErrNone) {
    if (!aStem_GetPacket(stemLib, 
    		         sCMPS03IICDataFilter,
    		         NULL,
    		         aCMPS03_GETDATATIMEOUTMS,
    		         &packet,
    		         &cmpsErr)) {
      if (!aPacket_GetData(stemLib, packet, &module,
          		   &length, data, &cmpsErr)) {
        if (length != 4)
          cmpsErr = aErrIO;
        else {
          short raw = aUtil_RetrieveShort(&data[2]);
          switch (units) {
          case aCMPS03_RADIANS:
            *pVal = (float)(((float)raw / 3600.0f) * 2.0f * aPI);
            break;
          case aCMPS03_DEGREES:
            *pVal = (float)((float)raw / 10.0f);
            break;
          case aCMPS03_ROTATIONS:
            *pVal = (float)((float)raw /3600.0f);
            break;
          } /* switch */
        }
      } else {
        /* if we timed out, there is not likely a CMPS03 present */
        if (cmpsErr == aErrTimeout)
          cmpsErr = aErrNotFound;
          
      }
    }
  }

  return cmpsErr;

} /* aCMPS03_GetRange */
