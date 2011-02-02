/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aHTTP.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Cross-Platform implementation of basic drawing     */
/*              routines.                                          */
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


#ifndef _aHTTP_H_
#define _aHTTP_H_

#include "aUI.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* maximum chunk is:
 *      5, 3 hex digits plus crlf) +
 *    800, 1024 byte OCTET plus crlf +
 *      3, 0 plus crlf
 *      2, crlf
 * =  810
 *
 * This should be less than 1024 total as we want to manage the
 * byte coelescing ourselves (Nagle is turned off below us.)
 */

#define aCHUNKBUFSIZE		810
#define aCHUNKMAXLEN		800
#define aCHUNKDATAOFFSET	5

#define aHTTPMAXHEADER		1000
#define aHTTPBUFFERINC		100
#define aHTTPNUMPORTS		10

typedef struct aBuffer {
  unsigned long		nBufferSize;
  char*			pBuffer;
} aBuffer;

typedef struct aHTTP {
  aIOLib		ioRef;
  unsigned short	nPort;
  unsigned long         nInetAddr;
  aStreamRef		ports[aHTTPNUMPORTS];
  int			nCur;
  aStreamRef		log;
  aHTTPRequestProc	requestProc;
  void*			vpRef;
  aMemPoolRef		paramPool;
  aSymbolTableRef 	assetCache;
  aStreamRef		headerBuffer;
  aStreamRef		replyBuffer;
  aSettingFileRef       settings;
  aBool                 bCreatedSettings;
  unsigned int		nTemplateBlock;
  int			check;
} aHTTP;

#define aHTTPCHECK	0xE11D

#define aVALIDHTTP(p)						   \
  if ((p == NULL) ||						   \
      (((aHTTP*)p)->check != aHTTPCHECK)) {		   	   \
    uiErr = aErrParam;						   \
  }


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static aErr sHTTP_LogLine(aHTTP* pHTTP, const char* line);
static aErr sHTTP_HandleRequest(aHTTP* pHTTP, const char* line);
static aErr sHTTP_RebuildStream(aHTTP* pHTTP, const int index);
static aErr sHTTP_HandleAsset(aHTTP* pHTTP, 
			      const char* name,
			      aStreamRef replyStream);
static aErr sHTTP_WriteTemplateBuffer(aHTTP* pHTTP,
			              const char* pData,
				      const unsigned int nLength,
				      const unsigned int nBlock,
			              aHTTPTemplateProc templateProc,
			              void* vpRef,
			              aStreamRef reply);
static aErr sHTTP_HandleTemplate(aHTTP* pHTTP,
			  	 aStreamRef tpl,
			  	 const char** remainder,
				 const unsigned int nBlock,
			         aHTTPTemplateProc templateProc,
			         void* vpRef,
			         aStreamRef reply);
static aErr sHTTP_ParseParams(aHTTP* pHTTP, 
			      const char* paramStr, 
			      aSymbolTableRef* pSymRef); 
static aErr sHTTP_FreeParamVal(void* pData, void* ref);
static aErr sHTTP_FreeAssetCache(void* pData, void* ref);
static aErr sHTTP_DisplayMsg(aHTTP* pHTTP,
			     aStreamRef reply,
			     const char* message);

#endif /* _aHTTP_H_ */

