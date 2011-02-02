/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aUSB.c                                               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of USB stream I/O routines for      */
/*		Unix.						   */
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

#ifdef aUNIX

#include <fcntl.h>
#include <unistd.h>

#include "unix_aIO.h"
#include "aStream.h"


typedef struct aUnixUSBStream {
  unsigned int		nSerialNum;
  int                   nUSBDevice;

  int			check;
  
} aUnixUSBStream;

#define aUNIXUSBCHECK  0xDEAD


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sUSBStreamGet(char* pData, void* ref);
static aErr sUSBStreamPut(char* pData, void* ref);
static aErr sUSBStreamWrite(const char* pData, 
			    const unsigned long nSize,
			    void* ref);
static aErr sUSBStreamDelete(void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sUSBStreamGet
 */

aErr 
sUSBStreamGet(char* pData,
	      void* ref)
{
  aUnixUSBStream* pStreamData = (aUnixUSBStream*)ref;

  if (pStreamData && (pStreamData->check == aUNIXUSBCHECK)) {
    int nRead = read(pStreamData->nUSBDevice, pData, 1);

    if (nRead == 1)
      return aErrNone;

    if (nRead == 0)
      return aErrNotReady;

    printf("sUSBStreamGet DEBUG error, nRead = %d\n", nRead);
    return aErrIO;
  }

  return aErrParam;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sUSBStreamPut
 */

aErr 
sUSBStreamPut(char* pData,
	      void* ref)
{
  aErr err = aErrNone;
  aUnixUSBStream* pStreamData = (aUnixUSBStream*)ref;

  if (pStreamData && (pStreamData->check == aUNIXUSBCHECK)) {
    if (write(pStreamData->nUSBDevice, pData, 1) != 1)
      err = aErrIO;
  } else 
    err = aErrParam;
  
  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sUSBStreamWrite
 */

aErr 
sUSBStreamWrite(const char* pData,
		const unsigned long nSize,
		void* ref)
{
  aErr err = aErrNone;
  aUnixUSBStream* pStreamData = (aUnixUSBStream*)ref;

  if (pStreamData && (pStreamData->check == aUNIXUSBCHECK)) {
    if (write(pStreamData->nUSBDevice, pData, nSize) != (int)nSize)
      err = aErrIO;
  } else 
    err = aErrParam;
  
  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sUSBStreamDelete
 */

aErr 
sUSBStreamDelete(void* ref)
{
  aErr err = aErrNone;
  aUnixUSBStream* pStreamData = (aUnixUSBStream*)ref;
  
  if (pStreamData && (pStreamData->check = aUNIXUSBCHECK)) {
    if (pStreamData->nUSBDevice) {
      close(pStreamData->nUSBDevice);
      pStreamData->nUSBDevice = 0;
    }
    aBZero(pStreamData, sizeof(aUnixUSBStream));
    aMemFree(pStreamData);
  }

  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateUSB
 */

aLIBRETURN 
aStream_CreateUSB(aIOLib ioRef, 
		  const unsigned int serialNum, 
		  aStreamRef* pStreamRef, 
		  aErr* pErr)
{
  aErr err = aErrNone;
  aUnixUSBStream* pStreamData = NULL;
  char devName[aFILE_NAMEMAXCHARS];
  aStreamRef usbStream;

  aVALIDIO(ioRef);
  if ((err == aErrNone) && (pStreamRef == NULL)) {
printf("invalid parameters\n");
    err = aErrParam;
  }

  /* allocate the private structure for the usb device */
  if (err == aErrNone) {
    pStreamData = (aUnixUSBStream*)aMemAlloc(sizeof(aUnixUSBStream));
    if (pStreamData == NULL) {
printf("unable to allocate\n");
      err = aErrMemory;
    } else {
      aBZero(pStreamData, sizeof(aUnixUSBStream));
      pStreamData->nSerialNum = serialNum;
      snprintf(devName, aFILE_NAMEMAXCHARS, 
	       "/dev/brainstem.%08X", serialNum);
      pStreamData->check = aUNIXUSBCHECK;
    }
  }

  /* try to build the generalized stream abstraction */
  if (err == aErrNone) {
    aStream_Create(ioRef, 
    		   sUSBStreamGet,
    		   sUSBStreamPut,
    		   sUSBStreamDelete,
    		   pStreamData, 
    		   &usbStream, 
    		   &err);
  }
  if (err != aErrNone) {
printf("unable to create stream\n");
    aMemFree(pStreamData);
    usbStream = NULL;
  }

  /* from here on, we can just destroy the stream to clean up */

  /* open the actual device */
  if (err == aErrNone) {
    pStreamData->nUSBDevice = open(devName, O_RDWR);
    if (pStreamData->nUSBDevice < 0) {
printf("unable to open \"%s\"\n", devName);
      err = aErrIO;
    }
  }

  /* the return values and patch in the optimized write routine */
  if (err == aErrNone) {
    aStream* pStream = (aStream*)usbStream;
    pStream->writeProc = sUSBStreamWrite;
    *pStreamRef = usbStream;
  } else {
    if (usbStream)
      aStream_Destroy(ioRef, usbStream, NULL);
  }
 
  if (pErr != NULL)
    *pErr = err;
  
  return(err != aErrNone);
}

#endif /* aUNIX */
