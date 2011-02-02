/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStreamUtil.h                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent BrainStem     */
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


#ifndef _aStreamUtil_H_
#define _aStreamUtil_H_

#include "aIO.h"
#include "aStem.h"
#include "aUI.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * defaults for port settings
 */

#define LINKTYPEKEY             "linktype"
#define DEFAULTLINKTYPE         "serial"
#define BAUDRATEKEY		"baudrate"
#define PORTNAMEKEY		"portname"
#define DEFAULTBAUDRATE		9600
#define USBSERIALKEY            "usb_id"
#define MODULEKEY		"module"
#define IPADDRKEY		"ip_address"
#define DEFAULTIPADDR		(unsigned long)0x7F000001
#define IPPORTKEY		"ip_port"
#define DEFAULTIPPORT		8000
#ifdef aWIN
#define DEFAULTPORTNAME 	"COM1"
#endif /* aWIN */
#ifdef aWINCE
#define DEFAULTPORTNAME 	"COM1:"
#endif /* aWINCE */
#ifdef aMAC
#define DEFAULTPORTNAME 	"printer"
#endif /* aMAC */
#ifdef aPALM
#define DEFAULTPORTNAME 	"serial"
#endif /* aPALM */
#ifdef aUNIX
#ifdef aMACX
#define DEFAULTPORTNAME 	"tty.usbserial"
#else /* aMACX */
#define DEFAULTPORTNAME 	"ttyS0"
#endif /* aMACX */
#endif /* aUNIX */ 

#ifdef __cplusplus
extern "C" {
#endif

aLIBRETURN aStreamUtil_CreateRawSettingStream(aIOLib ioRef,
					      aUILib uiRef,
				     	      aSettingFileRef settings,
				     	      aStreamRef* pStreamRef,
				     	      aErr* pErr);

aLIBRETURN aStreamUtil_CreateSettingStream(aIOLib ioRef,
					   aUILib uiRef,
				     	   aStemLib stemRef,
				     	   aSettingFileRef settings,
				     	   aStreamRef* pStreamRef,
				     	   aErr* pErr);

#ifdef __cplusplus
}
#endif

#endif /* _aStreamUtil_H_ */
