/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aBrainStem.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent and 	   */
/*		BrainStem PPRK communication layer.		   */
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
#include "aBrainStem.h"
#include "aUtil.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

aBool sBrainStem_InitPacketFilter(const unsigned char address,
				  const unsigned char length,
				  const char* data,
				  void* ref);
aBool sBrainStem_A2DPacketFilter(const unsigned char address,
				 const unsigned char length,
				 const char* data,
				 void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBrainStem_InitPacketFilter
 */

aBool sBrainStem_InitPacketFilter(const unsigned char address,
				  const unsigned char length,
				  const char* data,
				  void* ref)
{
  if (length == 0)
    return aTrue;

  return aFalse;

} /* sBrainStem_InitPacketFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBrainStem_A2DPacketFilter
 */

aBool sBrainStem_A2DPacketFilter(const unsigned char address,
				 const unsigned char length,
				 const char* data,
				 void* ref)
{
  if ((length == 4) &&
      (data[0] == cmdDEV_VAL))
    return aTrue;

  return aFalse;

} /* sBrainStem_A2DPacketFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBrainStem_Init
 * 
 * This routine establishes what the address of the stem is and 
 * stores it in the module.
 */

aErr aBrainStem_Init(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;
  unsigned char address;
#ifdef aDEBUG
  unsigned char length = 0;
#endif /* aDEBUG */

  aAssert(pPPRK);
  aAssert(pPPRK->linkStream);

  /* try to set up the brainstem library if it is not in place */
  if ((pprkErr == aErrNone) &&
      (pPPRK->controller.brainstem.stemRef == NULL)) {
    aStem_GetLibRef(&pPPRK->controller.brainstem.stemRef, &pprkErr);
    if (pprkErr == aErrNone) {
      aStem_SetStream(pPPRK->controller.brainstem.stemRef,
      		      pPPRK->linkStream, 
      		      kStemModuleStream,
      		      &pprkErr);
    }
  }

  /* now try to establish whether we have the address or not yet */
  if ((pprkErr == aErrNone) && 
      (pPPRK->controller.brainstem.stemRef != NULL) &&
      (pPPRK->controller.brainstem.bInited == aFalse)) {

    if (pprkErr == aErrNone) {
      int module;
      aSettingFile_GetInt(pPPRK->ioRef, pPPRK->settingFile,
    			 "module", 
    			 &module, 2, &pprkErr);
      if (pprkErr == aErrNone)
        address = (unsigned char)module;
    }

    if (pprkErr == aErrNone) {
      /* we should have a zero-length reply packet because
       * of the filter
       */
      aAssert(length == 0);
      
      /* we should have a valid IIC address */
      aAssert(address != 0);
      aAssert(!(address & 0x01));
      
      /* store the returned address and set the 
       * initialization flag */
      pPPRK->controller.brainstem.address = address;
      pPPRK->controller.brainstem.bInited = aTrue;
    }
  }

  return pprkErr;

} /* aBrainStem_Init */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBrainStem_TimeSlice
 */

aErr aBrainStem_TimeSlice(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;

  /* ensure we are properly initialized */
  if ((pprkErr == aErrNone) && 
      (pPPRK->controller.brainstem.bInited == aFalse)) {
    pprkErr = aBrainStem_Init(pPPRK);
  }

  if (pprkErr == aErrNone) {
    aPacketRef packet;
    aStem_GetPacket(pPPRK->controller.brainstem.stemRef,
      		    NULL, NULL, 10,
      		    &packet, &pprkErr);
    if (pprkErr == aErrTimeout)
      pprkErr = aErrNone;
    else if (pprkErr == aErrNone) {
      aPacket_Destroy(pPPRK->controller.brainstem.stemRef, 
        	      packet, &pprkErr);
    }
  }

  return pprkErr;

} /* aBrainStem_TimeSlice */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBrainStem_SetServo
 */

aErr aBrainStem_SetServo(aPPRK* pPPRK,
			 unsigned int nServoNum,
			 unsigned int nAbsolutePos)
{
  aErr pprkErr = aErrNone;

  /* ensure we are properly initialized */
  if ((pprkErr == aErrNone) && 
      (pPPRK->controller.brainstem.bInited == aFalse)) {
    pprkErr = aBrainStem_Init(pPPRK);
  }

  if (pprkErr == aErrNone) {
    char data[aSTEMMAXPACKETBYTES];
    aPacketRef outPacket;

    /* we shouldn't get here without the stem being inited */    
    aAssert(pPPRK->controller.brainstem.bInited == aTrue);

    /* build up the packet that tells the servo to change */
    data[0] = (char)cmdSRV_ABS;
    data[1] = (char)nServoNum;
    data[2] = (char)nAbsolutePos;
    if (!aPacket_Create(pPPRK->controller.brainstem.stemRef, 
  		        pPPRK->controller.brainstem.address, 
  		        3, data, &outPacket, &pprkErr)) {
      aStem_SendPacket(pPPRK->controller.brainstem.stemRef, 
      		       outPacket, &pprkErr);
    }
  }

  return pprkErr;

} /* aBrainStem_SetServo */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBrainStem_GetRange
 */

aErr aBrainStem_GetRange(aPPRK* pPPRK,
			 unsigned int nRangeNum,
			 float* pValue)
{
  aErr pprkErr = aErrNone;
  unsigned char address;
  unsigned char length;
  char data[aSTEMMAXPACKETBYTES];

  /* ensure we are properly initialized */
  if ((pprkErr == aErrNone) && 
      (pPPRK->controller.brainstem.bInited == aFalse)) {
    pprkErr = aBrainStem_Init(pPPRK);
  }

  /* now try to get the range data */
  if ((pprkErr == aErrNone) && 
      (pPPRK->controller.brainstem.stemRef != NULL) &&
      (pPPRK->controller.brainstem.bInited == aTrue)) {

    /* send the magic byte to get the address back */
    aPacketRef outPacket;
    aPacketRef reply;
 
    /* build up the packet */
    data[0] = (char)(cmdA2D_RD);
    data[1] = (char)(nRangeNum | maskDEV_VAL_HOST);

    if (!aPacket_Create(pPPRK->controller.brainstem.stemRef, 
  		        pPPRK->controller.brainstem.address, 
  		        2, data, &outPacket, &pprkErr)) {
      aStem_SendPacket(pPPRK->controller.brainstem.stemRef, 
      		       outPacket, &pprkErr);
    }

    if (pprkErr == aErrNone) {    
      /* get the reply packet to find the address */
      aStem_GetPacket(pPPRK->controller.brainstem.stemRef,
      		      sBrainStem_A2DPacketFilter,
      		      pPPRK, aPPRKSTEMTIMEOUTMS,
      		      &reply, &pprkErr);
    }

    /* get the data from the reply packet (it contains 
     * the range data
     */
    if (pprkErr == aErrNone) {
      aPacket_GetData(pPPRK->controller.brainstem.stemRef,
      		      reply, &address, &length, data, &pprkErr);
      
      /* toss the packet now that we have the data */
      if (pprkErr == aErrNone) {
        aPacket_Destroy(pPPRK->controller.brainstem.stemRef, 
        		reply, &pprkErr);
      }
    }
    
    if (pprkErr == aErrNone) {
      unsigned short raw;

      /* we should have a device data packet because
       * of the filter
       */
      aAssert(length == 4);
      aAssert(data[0] == cmdDEV_VAL);
      aAssert(data[1] == (char)nRangeNum);

      /* get out the a/d data and normalize (0.0 - 1.0) */
      raw = aUtil_RetrieveUShort(&data[2]);

      /* the value is packed into the top 10 bits of the short */
      raw >>= 6;
      *pValue = (1024 - raw) / 1024.0f;
    }
  }

  return pprkErr;

} /* aBrainStem_GetRange */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBrainStem_Destroy
 *
 * cleans up the stem library when done
 */

aErr aBrainStem_Destroy(aPPRK* pPPRK)
{
  aErr pprkErr = aErrNone;
  
  aAssert(pPPRK);

  if ((pprkErr == aErrNone) &&
      (pPPRK->controller.brainstem.stemRef != NULL)) {
    aStem_ReleaseLibRef(pPPRK->controller.brainstem.stemRef, 
    			&pprkErr);
    /* this destroys the link stream so set the record of it
     * to null */
    pPPRK->linkStream = NULL;
    pPPRK->controller.brainstem.stemRef = NULL;
    pPPRK->controller.brainstem.address = 0;
  }

  return pprkErr;

} /* aBrainStem_Destroy */
