/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStreamUtil.c                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent BrainStem */
/*		stream initialization.				   */
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

#include "aStreamUtil.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStreamUtil_CreateRawSettingStream
 */

aLIBRETURN aStreamUtil_CreateRawSettingStream(
  aIOLib ioRef,
  aIOLib uiRef,
  aSettingFileRef settings,
  aStreamRef* pStreamRef,
  aErr* pErr
)
{
  aErr suErr = aErrNone;
  aStreamRef linkStream = NULL;
  char* linktype;

  if (suErr == aErrNone)
    aSettingFile_GetString(ioRef, 
    			   settings, 
    			   LINKTYPEKEY, 
    			   &linktype,
     			   DEFAULTLINKTYPE, 
     			   &suErr);

  if (suErr == aErrNone) {
    if (!aStringCompare(linktype, "serial")) {
      char* portname;
      int baudRate;
      
      if (suErr == aErrNone)
	aSettingFile_GetInt(ioRef, 
			    settings, 
			    BAUDRATEKEY, 
			    &baudRate,
			    DEFAULTBAUDRATE, 
			    &suErr);
      
      if (suErr == aErrNone)
	aSettingFile_GetString(ioRef, 
			       settings, 
			       PORTNAMEKEY, 
			       &portname,
			       DEFAULTPORTNAME, 
			       &suErr);
      if (suErr == aErrNone) {
	aStream_CreateSerial(ioRef,
			     portname,
			     (unsigned int)baudRate,
			     &linkStream,
			     &suErr);
	if (suErr == aErrBusy)
	  aDialog_Message(uiRef,
        		"Serial Port Is Busy",
			  NULL, NULL, NULL);
	else if (suErr == aErrNotFound)
	  aDialog_Message(uiRef,
			  "Serial Port Not Found",
			  NULL, NULL, NULL);
	else if (suErr == aErrIO)
	  aDialog_Message(uiRef,
			  "Unknown Serial Port Error",
			  NULL, NULL, NULL);
      }

    } else if (!aStringCompare(linktype, "usb")) {
      int serialnum;
      
      if (suErr == aErrNone)
	aSettingFile_GetInt(ioRef, 
			    settings, 
			    USBSERIALKEY, 
			    &serialnum,
			    0, 
			    &suErr);
      if (suErr == aErrNone) {
	if (serialnum) {
	  aStream_CreateUSB(ioRef, serialnum, &linkStream, &suErr);

	  /* error description here? */

	} else {
	  aDialog_Message(uiRef,
			  "USB ID not set in config file",
			  NULL, NULL, NULL);
	}
      }

    } else if (!aStringCompare(linktype, "ip")) {
      unsigned long address;
      int port;
      if (suErr == aErrNone) {
        aSettingFile_GetInetAddr(ioRef, settings, 
    			         IPADDRKEY, &address,
     			         DEFAULTIPADDR, &suErr);
      }
      if (suErr == aErrNone) {
        aSettingFile_GetInt(ioRef, settings, 
    			    IPPORTKEY, &port,
     			    DEFAULTIPPORT, &suErr);
      }
      aStream_CreateSocket(ioRef,
    			   address,
    			   (unsigned short)port,
    			   aFalse,
    			   &linkStream,
    			   &suErr);
    }
  }

  *pStreamRef = (suErr == aErrNone) ? linkStream : NULL;
  
  if (pErr != NULL)
    *pErr = suErr;
  
  return (aLIBRETURN)(suErr != aErrNone);
  
} /* aStreamUtil_CreateRawSettingStream */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStreamUtil_CreateSettingStream
 */

aLIBRETURN aStreamUtil_CreateSettingStream(
  aIOLib ioRef,
  aIOLib uiRef,
  aStemLib stemRef,
  aSettingFileRef settings,
  aStreamRef* pStreamRef,
  aErr* pErr
)
{
  aErr suErr = aErrNone;
  aStreamRef linkStream = NULL;

  if (suErr == aErrNone)
    aStreamUtil_CreateRawSettingStream(ioRef, uiRef, settings, 
    				       &linkStream, &suErr);

  /* set the stream */
  if ((suErr == aErrNone) &&
      (linkStream != NULL)) {
    aStem_SetStream(stemRef, 
    		    linkStream, 
    		    kStemModuleStream,
    		    &suErr);
  }

  if (suErr == aErrNone) {
    *pStreamRef = linkStream;
  } else if (linkStream) {
    aStream_Destroy(ioRef, linkStream, NULL);
  }

  if (pErr != NULL)
    *pErr = suErr;
  
  return (aLIBRETURN)(suErr != aErrNone);

} /* aStreamUtil_CreateSettingStream */

