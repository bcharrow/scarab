/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsole_Cmds.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent Console   */
/*		command handlers.				   */
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

#include "aConsole_Cmds.h"
#include "aConsole_Tests.h"
#include "aConsoleText.h"
#include "aCmd.tea"
#include "aUtil.h"
#include "aSteep.h"
#include "aVersion.h"
#include "aSP03.h"
#include "aStreamUtil.h"
#include "aBrainDump.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines
 */

static aErr sConsole_InputFileParam(aConsole* pConsole,
				    aTokenizerRef tokenizer,
			            const aFileArea eArea,
				    aStreamRef* pFileStream,
				    const aBool bFinal,
				    char* filename_copy,
				    const char* default_ext);
static aErr sConsole_OutputFileParam(aConsole* pConsole,
				     aTokenizerRef tokenizer,
			             const aFileArea eArea,
				     aStreamRef* pFileStream,
				     const aBool bFinal,
				     char* filename_copy);
static aErr sConsole_ModuleParam(aConsole* pConsole,
				 aTokenizerRef tokenizer,
				 unsigned char* pModuleNum,
				 aBool bFinal);
static aErr sConsole_UShortParam(aConsole* pConsole,
			       aTokenizerRef tokenizer,
			       unsigned short* pPortNum,
			       aBool bFinal);

static aErr sConsole_VMLaunch(aConsole* pConsole, 
			      unsigned char module, 
			      aTEAProcessID pid);

static aBool sLaunchVMFilter(const unsigned char module,
		     	     const unsigned char dataLength,
		      	     const char* data,
		      	     void* ref);

static aErr sShowStatus(const char* pText,
			const void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sShowStatus
 */

aErr sShowStatus (
  const char* pText,
  const void* ref
)  
{
  aErr err = aErrNone;
  aConsole* pConsole = (aConsole*)ref;

  aAssert(pConsole);
  if (pText)
    err = aConsole_DisplayLine(pConsole, pText, kDisplayStatus);
  
  return err;

} /* sShowFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLaunchVMFilter
 */

aBool sLaunchVMFilter(const unsigned char module,
		     	   const unsigned char dataLength,
		      	   const char* data,
		      	   void* ref)
{
  int temp = (int)(long)ref;
  if ((data[0] == cmdVM_RUN)
      && (module == (unsigned char)temp))
    return aTrue;

  return aFalse;

} /* sLaunchVMFilter */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_InputFileParam
 */

aErr sConsole_InputFileParam(aConsole* pConsole,
			     aTokenizerRef tokenizer,
			     const aFileArea eArea,
			     aStreamRef* pFileStream,
			     const aBool bFinal,
			     char* filename_copy,
			     const char* default_ext)
{
  aErr consoleErr = aErrNone;
  aToken* pToken = NULL;
  char filename[aMAXIDENTIFIERLEN];
  char rootname[aMAXIDENTIFIERLEN];
  aStreamRef fileStream = NULL;

  aVALIDCONSOLE(pConsole);
  
  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_MISSING_FILENAME, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {
    /* here we have a token but don't know its type */
    if (pToken->eType == tkString) {
      aStringCopySafe(filename, aMAXIDENTIFIERLEN, pToken->v.string);
    } else {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_STRING, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
    /* we are done with the token so release it */
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  if ((bFinal == aTrue) && (consoleErr == aErrNone)) {
    /* here we have a file name, check for extra input */
    if (!aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
      pToken = NULL;
      aConsole_DisplayLine(pConsole, aCMD_UNEXPECTED_PARAM, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  /* MRW 10-26-2001 */
  /* if we have a default extension then  */
  /* check file name for default extension */
  /* if it does not have it, then add it   */
  if ((consoleErr == aErrNone)
      && (default_ext != NULL)) {
    aString_GetFileRoot(rootname, filename);
    if (aStringCompare(rootname, filename) == 0) {
      aStringCatSafe(filename, aMAXIDENTIFIERLEN, default_ext);
    }
  }

  if ((consoleErr == aErrNone) 
      && (pFileStream != NULL)) {
    /* here we are good to go, create the input stream */
    if (aStream_CreateFileInput(pConsole->ioLib, 
    		   		filename, 
    		   		eArea,
    		   		&fileStream,
    				&consoleErr)) {
      switch(consoleErr) {
      case aErrNotFound:
        aConsole_DisplayLine(pConsole, aCMD_FILE_NOT_FOUND, 
        		     kDisplayStatus);
        break;
      default:
        aConsole_DisplayLine(pConsole, aCMD_FILE_ERROR, 
        		     kDisplayStatus);
        break;
      }
    }
    if (consoleErr == aErrNone)
      *pFileStream = fileStream;
  }

  /* copy the filename if it was requested */
  if ((consoleErr == aErrNone) 
      && (filename_copy != NULL)) {
    aStringCopySafe(filename_copy, aMAXIDENTIFIERLEN, filename);
  }

  if (pToken != NULL)
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);

  return consoleErr;

} /* sConsole_InputFileParam */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_OutputFileParam
 */

aErr sConsole_OutputFileParam(aConsole* pConsole,
			      aTokenizerRef tokenizer,
			      const aFileArea eArea,
			      aStreamRef* pFileStream,
			      const aBool bFinal,
			      char* filename_copy)
{
  aErr consoleErr = aErrNone;
  aToken* pToken = NULL;
  char filename[aMAXIDENTIFIERLEN];
  aStreamRef fileStream = NULL;

  aVALIDCONSOLE(pConsole);
  
  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_MISSING_FILENAME, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {
    /* here we have a token but don't know its type */
    if (pToken->eType == tkString) {
      aStringCopySafe(filename, aMAXIDENTIFIERLEN, pToken->v.string);
    } else {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_STRING, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
    /* we are done with the token so release it */
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  if ((bFinal == aTrue) && (consoleErr == aErrNone)) {
    /* here we have a file name, check for extra input */
    if (!aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
      pToken = NULL;
      aConsole_DisplayLine(pConsole, aCMD_UNEXPECTED_PARAM, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {
    /* here we are good to go, create the input stream */
    if (filename_copy != NULL)
      aStringCopySafe(filename_copy, aMAXIDENTIFIERLEN, filename);
    if (aStream_CreateFileOutput(pConsole->ioLib, 
    		   		 filename, 
    		   		 eArea,
    		   		 &fileStream,
    				 &consoleErr)) {
      switch(consoleErr) {
      case aErrNotFound:
        aConsole_DisplayLine(pConsole, aCMD_FILE_NOT_FOUND, 
        		     kDisplayStatus);
        break;
      default:
        aConsole_DisplayLine(pConsole, aCMD_FILE_ERROR, 
        		     kDisplayStatus);
        break;
      }
    }
  }

  if (consoleErr == aErrNone)
    *pFileStream = fileStream;
    
  if (pToken != NULL)
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);

  return consoleErr;

} /* sConsole_OutputFileParam */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_ModuleParam
 */

aErr sConsole_ModuleParam(aConsole* pConsole,
			  aTokenizerRef tokenizer,
			  unsigned char* pModuleNum,
			  aBool bFinal)
{
  aErr consoleErr = aErrNone;
  aToken* pToken = NULL;
  aBool bHost = aFalse;

  aVALIDCONSOLE(pConsole);
  
  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_MISSING_MODULE, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {

    /* here we have a token but don't know its type */
    if (pToken->eType == tkInt) {
      *pModuleNum = (unsigned char)pToken->v.integer;
    } else if ((pToken->eType == tkIdentifier)
               && (!aStringCompare("host", pToken->v.identifier))) {
      bHost = aTrue;
    } else {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_NUMBER, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }

    /* we are done with the token so release it */
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  if ((bFinal == aTrue) && (consoleErr == aErrNone)) {

    /* here we have a module reference, check for extra input */
    if (!aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
      pToken = NULL;
      aConsole_DisplayLine(pConsole, aCMD_UNEXPECTED_PARAM, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {
    /* here we are good to go, make sure module ref is valid */
    if (bHost) {
      *pModuleNum = 0;
    } else if ((*pModuleNum & 0x01) 
               || (*pModuleNum == 0)) {
      aConsole_DisplayLine(pConsole, aCMD_INVALID_MODULE, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }
    
  if (pToken != NULL)
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);

  return consoleErr;

} /* sConsole_ModuleParam */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_UShortParam
 */

aErr sConsole_UShortParam(aConsole* pConsole,
			aTokenizerRef tokenizer,
			unsigned short* pPortNum,
			aBool bFinal)
{
  aErr consoleErr = aErrNone;
  aToken* pToken = NULL;

  aVALIDCONSOLE(pConsole);
  
  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_MISSING_MODULE, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {

    /* here we have a token but don't know its type */
    if (pToken->eType == tkInt) {
      *pPortNum = (unsigned short)pToken->v.integer;
    } else {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_NUMBER, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }

    /* we are done with the token so release it */
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  if ((bFinal == aTrue) && (consoleErr == aErrNone)) {

    /* here we have a port reference, check for extra input */
    if (!aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
      pToken = NULL;
      aConsole_DisplayLine(pConsole, aCMD_UNEXPECTED_PARAM, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }
    
  if (pToken != NULL)
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);

  return consoleErr;

} /* sConsole_UShortParam */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleExit
 */

aErr aConsole_HandleExit(aConsole* pConsole,
			 aToken* pFirstToken,
			 aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aToken* pToken;
  
  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone) {
    if (!aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_UNEXPECTED_PARAM, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    } else
      pConsole->bDone = aTrue;
  }
  
  return consoleErr;

} /* aConsole_HandleExit */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleBatch
 */

aErr aConsole_HandleBatch(aConsole* pConsole,
			  aToken* pFirstToken,
			  aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char batchfile[aMAXIDENTIFIERLEN];

  aVALIDCONSOLE(pConsole);

  /* batch takes a single filename parameter */
  if (consoleErr == aErrNone)
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaUser, 
    					 NULL,
    					 aFalse,
    					 batchfile,
    					 NULL);

  /* use standard batch handling */
  if (consoleErr == aErrNone) {
    consoleErr = aConsole_Batch(pConsole, 
    				batchfile, 
    				aFileAreaUser, 
    				aTrue);
  }

  return consoleErr;

} /* aConsole_HandleBatch */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_VMLaunch
 */

aErr sConsole_VMLaunch(aConsole* pConsole, 
		       unsigned char module, 
		       aTEAProcessID pid)
{
  aErr consoleErr = aErrNone;

  aAssert(pConsole);

  if (pConsole->nDebugLevel > 0) {
    char num[10];

    /* build up the status string */
    consoleErr = aConsole_BufferOutput(pConsole, 
    				       aVM_PROCESS_LAUNCH);
    
    if (consoleErr == aErrNone)
      consoleErr = aConsole_BufferOutput(pConsole, ": ");

    if (consoleErr == aErrNone) {
      if (module == 0) {
        consoleErr = aConsole_BufferOutput(pConsole, aT_HOST_NAME);
      } else {
        aStringFromInt(num, module);
        consoleErr = aConsole_BufferOutput(pConsole, num);
      }
    }

    if (consoleErr == aErrNone)
      consoleErr = aConsole_BufferOutput(pConsole, ",");
 
    if (consoleErr == aErrNone) {
      aStringFromInt(num, pid);
      consoleErr = aConsole_BufferOutput(pConsole, num);
    }

    if (consoleErr == aErrNone)
      consoleErr = aConsole_DisplayBuffer(pConsole, 
      					  kDisplayStatus);
  }
  
  return consoleErr;

} /* sConsole_VMLaunch */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleLaunch
 */

aErr aConsole_HandleLaunch(aConsole* pConsole,
			   aToken* pFirstToken,
			   aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char filename[aMAXIDENTIFIERLEN];
  char rootname[aMAXIDENTIFIERLEN];
  char inputData[MAXINPUTLINE];
  aToken* pToken = NULL;
  unsigned char inputDataLen;
  aTEAProcessID pid;
  unsigned char slot = 0;
  unsigned char module = 0;

  aVALIDCONSOLE(pConsole);

  /* we don't parse packets in other modes */
  if ((consoleErr == aErrNone)
      && (pConsole->nMode != modeBrainStem)) {
    aConsole_DisplayLine(pConsole, 
    			 aCMD_INVALID_MODE, 
    			 kDisplayStatus);
    consoleErr = aErrParse;
  }

  if (consoleErr == aErrNone) {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_NO_PROGRAM_SPECIFIED, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {
    /* strings mean local, host vm code */
    if (pToken->eType == tkString) {
    
      aStringCopySafe(filename, aMAXIDENTIFIERLEN, pToken->v.string);
      
      /* MRW -- 11-28-01 default extension for launch file is .cup */
      aString_GetFileRoot(rootname, filename);
      if (aStringCompare(rootname, filename) == 0) {
        aStringCatSafe(filename, aMAXIDENTIFIERLEN, ".cup");
      }

    /* integers mean module code */
    } else if (pToken->eType == tkInt) {
      module = (unsigned char)pToken->v.integer;
      if ((module & 0x01) 
          || (module == 0)) {
        aConsole_DisplayLine(pConsole, aCMD_INVALID_MODULE, 
        		     kDisplayStatus);
        consoleErr = aErrParse;
      }
 
    /* anything else is a bad module specification */
    } else {
      aConsole_DisplayLine(pConsole, aCMD_INVALID_MODULE, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  /* get the slot number if we are loading to a module */
  if ((consoleErr == aErrNone)
      && (module != 0)
      && !aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
    
    /* the slot number */
    if (pToken->eType == tkInt) {
      slot = (unsigned char)pToken->v.integer;
 
    /* anything else is a bad slot specification */
    } else {
      aConsole_DisplayLine(pConsole, aCMD_SLOT_INVALID, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  /* pick up any launch data */
  if (consoleErr == aErrNone) {
    consoleErr = aConsole_ParseRaw(pConsole, 
    				   tokenizer,
    				   inputData,
    				   &inputDataLen,
    				   MAXINPUTLINE);
  }

  /* do the actual launch */
  if (consoleErr == aErrNone) {
    if (module == 0) {
      consoleErr = aConsole_Launch(pConsole,
    				   filename,
    				   aFileAreaObject,
    				   inputData,
    				   inputDataLen,
    				   aConsole_VMExit,
    				   pConsole,
    				   0,
    				   &pid);
    } else {
      aPacketRef packet;
      unsigned char length;
      unsigned char address;
      char* p = inputData;
      tSTACK packetDataLen;
      tSTACK packetDataLeft = inputDataLen;
      int numPackets = 0;
      char packetData[aSTEMMAXPACKETBYTES];

      packetData[0] = cmdVM_RUN;

      while ((consoleErr == aErrNone)
             && ((numPackets == 0)
                 || (packetDataLeft > 0))) {

        /* clear out the flag byte */
        length = 3;

        if (numPackets == 0) {
          /* set the first packet bit */
          packetData[1] = bitVM_RUN_FIRST;
          packetData[2] = (char)slot;

        } else {
          /* packets beyond first need process id */
          packetData[1] = bitVM_RUN_PID;
          packetData[2] = (char)pid;
        }

        /* add in any data */
        if (packetDataLeft > 0) {

          /* stuff as much data as possible */
          packetDataLen = (unsigned char)
          			(aSTEMMAXPACKETBYTES - length);
          if (packetDataLeft < packetDataLen) {
            packetDataLen = packetDataLeft;
            packetDataLeft = 0;
          } else {
            packetDataLeft -= packetDataLen;
          }
        
          if (packetDataLen > 0) {
            aMemCopy(&packetData[length], p, packetDataLen);
            p += packetDataLen;
            length += (unsigned char)packetDataLen;
          }
        }
 
        if (packetDataLeft == 0)
          packetData[1] |= bitVM_RUN_LAST;
    
        /* now, build up the packet */
        aPacket_Create(pConsole->stemLib, module,
    		       length, packetData, &packet, &consoleErr);
    
        /* send it */
        if (consoleErr == aErrNone)
          aStem_SendPacket(pConsole->stemLib, packet, &consoleErr);
    
        /* wait for the reply */
        if (consoleErr == aErrNone) {
	  int temp = module;
          aStem_GetPacket(pConsole->stemLib,
      			  sLaunchVMFilter,
      			  (void*)temp,
      			  pConsole->nMSTimeout,
      			  &packet, 
      			  &consoleErr);
	}
 
        /* get the reply packet data */
        if (consoleErr == aErrNone)
          aPacket_GetData(pConsole->stemLib, packet, 
        		  &address,
      		          &length, 
      		          packetData, 
      		          &consoleErr);

        /* snag the new process ID for new process */
        if (consoleErr == aErrNone) {
          aAssert(length == 2);
          aAssert(packetData[0] == cmdVM_RUN);
          pid = (aTEAProcessID)packetData[1];
          numPackets++;
        }

        /* clean up the reply packet */
        if (consoleErr == aErrNone)
          aPacket_Destroy(pConsole->stemLib, 
          		  packet, &consoleErr);
      } /* while */
    }
  }

  if (consoleErr == aErrNone)
    consoleErr = sConsole_VMLaunch(pConsole, module, pid);

  return consoleErr;

} /* aConsole_HandleLaunch */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleDsm
 */

aErr aConsole_HandleDsm(aConsole* pConsole,
			aToken* pFirstToken,
			aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char filename[aMAXIDENTIFIERLEN];
  char destfile[aMAXIDENTIFIERLEN];

  aVALIDCONSOLE(pConsole);

  /* just try to get the file name */
  if (consoleErr == aErrNone)
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaObject,
    					 NULL,
    					 aTrue,
    					 filename,
    					 ".cup");

  if (consoleErr == aErrNone) {
    aString_GetFileRoot(destfile, filename);
    aStringCatSafe(destfile, aMAXIDENTIFIERLEN, ".dsm");
    consoleErr = aConsole_Compile(pConsole, 
    				  fSteepDisassemble, 
    		   		  pConsole->consoleOutputRef,
    		 		  pConsole->consoleOutputRef,
    				  filename, 
    				  aFileAreaObject,
    				  destfile,
    				  aFileAreaUser);
  }

  return consoleErr;

} /* aConsole_HandleDsm */


#ifdef aTESTS
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleTest
 *
 * Run destination can be either the symbol host or an IIC address.
 */

aErr aConsole_HandleTest(aConsole* pConsole,
			 aToken* pFirstToken,
			 aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aStreamRef testStream;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone)
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaTest,
    					 &testStream,
    					 aTrue,
    					 NULL,
    					 NULL);

  if (consoleErr == aErrNone)
    consoleErr = aConsole_CreateTests(pConsole,
    				      testStream);

  return consoleErr;

} /* aConsole_HandleTest */
#endif /* aTESTS */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleLoad
 */

aErr aConsole_HandleLoad(aConsole* pConsole,
			 aToken* pFirstToken,
			 aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aStreamRef fileStream = NULL;
  aToken* pToken = NULL;
  char filename[aMAXIDENTIFIERLEN];
  unsigned char module;
  unsigned char slot = 0;

  aVALIDCONSOLE(pConsole);

  /* first, get the filename to copy */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaObject,
    					 &fileStream,
    					 aFalse,
    					 filename,
    					 ".cup");
  }
  
  /* now, get the module number */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_ModuleParam(pConsole, 
    				      tokenizer, 
    				      &module, 
    				      aFalse);
  }
  
  /* finally, get the slot */
  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_NUMBER, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {

    /* here we have a token but don't know its type */
    if (pToken->eType == tkInt) {
      slot = (unsigned char)pToken->v.integer;
    } else {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_NUMBER, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }

    /* we are done with the token so release it */
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  /* if we make it here, we are ready to actually copy the 
   * file to the module
   */
 
  if (consoleErr == aErrNone)
    consoleErr = aConsole_Load(pConsole,
    			       fileStream,
    			       module,
    			       slot);

  /* clean up the file either way if it was opened */
  if (fileStream != NULL) {
    aStream_Destroy(aStreamLibRef(fileStream), fileStream, NULL);
    fileStream = NULL;
  }

  return consoleErr;

} /* aConsole_HandleLoad */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleUnLoad
 */

aErr aConsole_HandleUnLoad(aConsole* pConsole,
			   aToken* pFirstToken,
			   aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aStreamRef fileStream = NULL;
  aStreamRef slotStream = NULL;
  aToken* pToken = NULL;
  char filename[aMAXIDENTIFIERLEN];
  unsigned char module;
  unsigned char slot = 0;

  aVALIDCONSOLE(pConsole);

  /* we don't parse packets in other modes */
  if ((consoleErr == aErrNone)
      && (pConsole->nMode != modeBrainStem)) {
    aConsole_DisplayLine(pConsole, 
    			 aCMD_INVALID_MODE, 
    			 kDisplayStatus);
    consoleErr = aErrParse;
  }

  /* get the module number */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_ModuleParam(pConsole, 
    				      tokenizer, 
    				      &module, 
    				      aFalse);
  }

  /* get the slot */
  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, &pToken, NULL)) {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_NUMBER, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {

    /* here we have a token but don't know its type */
    if (pToken->eType == tkInt) {
      slot = (unsigned char)pToken->v.integer;
    } else {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_NUMBER, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }

    /* we are done with the token so release it */
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  /* get the filename to copy */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_OutputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaObject,
    					 &fileStream,
    					 aTrue,
    					 filename);
  }

  /* if we make it here, we are ready to actually copy the 
   * file from the module
   */

  /* try to build a connection to the slot on the module */
  if (consoleErr == aErrNone) {
    aStem_CreateTEAFileInput(pConsole->stemLib,
    			     module,
    			     slot,
    			     &slotStream, 
    			     &consoleErr);
  }
  
  /* do the actual unloading */
  if (consoleErr == aErrNone) {
    aErr readErr = aErrNone;
    aErr writeErr = aErrNone;
    char byte;
    
    /* step through and write the bytes */
    while ((readErr == aErrNone) &&
           (writeErr == aErrNone)) {
      if (!aStream_Read(aStreamLibRef(fileStream), slotStream, 
      			&byte, 1, &readErr)) {
        aStream_Write(aStreamLibRef(slotStream), fileStream,
        	      &byte, 1, &writeErr);
      }
    } /* while no errors */ 

    /* now determine whether we succeeded or not */
    if ((readErr == aErrEOF) && 
         (writeErr == aErrNone)) {
      if (pConsole->nDebugLevel > 0)
        consoleErr = aConsole_DisplayLine(pConsole, 
        				  aCMD_UNLOAD_SUCCEEDED, 
        				  kDisplayStatus);
    } else {
      aConsole_DisplayLine(pConsole, aCMD_UNLOAD_FAILED, 
      			   kDisplayStatus);
      consoleErr = aErrIO;
    }


  }

  /* clean up the file either way if it was opened */
  if (fileStream != NULL) {
    aStream_Destroy(aStreamLibRef(fileStream), fileStream, NULL);
    fileStream = NULL;
  }

  /* clean up the slot stream either way if it was opened */
  if (slotStream != NULL) {
    aStream_Destroy(aStreamLibRef(slotStream), slotStream, NULL);
    slotStream = NULL;
  }

  return consoleErr;

} /* aConsole_HandleUnLoad */


#ifdef aDEBUGGER
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleDebug
 */

aErr aConsole_HandleDebug(aConsole* pConsole,
			  aToken* pFirstToken,
			  aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char srcName[aFILE_NAMEMAXCHARS];
  char cupName[aFILE_NAMEMAXCHARS];
  char dsmName[aFILE_NAMEMAXCHARS];
  char inputData[MAXINPUTLINE];
  unsigned char inputDataLen;
  unsigned char module;
  unsigned int id;
#if 1
  unsigned long inetAddr;
#endif

  aVALIDCONSOLE(pConsole);

  /* we don't parse packets in other modes */
  if ((consoleErr == aErrNone)
      && (pConsole->nMode != modeBrainStem)) {
    aConsole_DisplayLine(pConsole, 
    			 aCMD_INVALID_MODE, 
    			 kDisplayStatus);
    consoleErr = aErrParse;
  }


  /* first, get the filename to debug */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaUser,
    					 NULL,
    					 aFalse,
    					 srcName,
    					 ".tea");
  }

  /* module number is next (it may not be final) */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_ModuleParam(pConsole, 
                                      tokenizer,
                                      &module,
                                      aFalse);
  }
 
  /* pick up any launch data */
  if (consoleErr == aErrNone) {
    consoleErr = aConsole_ParseRaw(pConsole, 
                                   tokenizer,
                                   inputData,
                                   &inputDataLen,
                                   MAXINPUTLINE);
  }
 
   /* build the necessary output file names */
  if (consoleErr == aErrNone) {
    aString_GetFileRoot(cupName, srcName);
    aStringCopy(dsmName, cupName);
    aStringCat(cupName, ".cup");
    aStringCat(dsmName, ".dsm");
  }

  /* go compile the file with debug symbols turned on to 
   * generate the debug file and a current cup file */
  if (consoleErr == aErrNone) {
    consoleErr = aConsole_Compile(pConsole, 
    				  fSteepGenerateCode 
    				  | fSteepGenerateSymbols,
    				  pConsole->consoleOutputRef,
    				  pConsole->consoleOutputRef,
    				  srcName,
    				  aFileAreaUser,
    				  cupName,
    				  aFileAreaObject);
    if (consoleErr == aErrNotFound) {
      aConsole_DisplayLine(pConsole, 
      			   aCMD_FILE_NOT_FOUND, kDisplayStatus);
    }
  }

  /* dissassemble the new cup file */
  if (consoleErr == aErrNone) {
    consoleErr = aConsole_Compile(pConsole, 
    				  fSteepDisassemble,
    				  pConsole->consoleOutputRef,
    				  pConsole->consoleOutputRef,
    				  cupName,
    				  aFileAreaObject,
    				  dsmName,
    				  aFileAreaUser);
    if (consoleErr == aErrNotFound) {
      aConsole_DisplayLine(pConsole, 
      			   aCMD_FILE_NOT_FOUND, kDisplayStatus);
    }
  }

  /* add the session to the debugger */
  if (consoleErr == aErrNone) {
    if (pConsole->pDebugger) {
      aTEAvmLaunchBlock lb;
      lb.data = inputData;
      lb.dataSize = inputDataLen;
      lb.exitProc = aConsole_VMExit;
      lb.exitRef = pConsole;
      lb.flags = fTEAvmDebug;
      consoleErr = aTEADebugger_AddSession(pConsole->pDebugger,
                                         srcName,
                                         module,
                                         aFileAreaUser,
                                         aFileAreaObject,
                                         &lb);
      id = lb.pid;
    } else {
      consoleErr = aConsole_DisplayLine(pConsole,
					"Unable to launch debugger",
					kDisplayStatus);
    }
  }

#if 1
  /* this isn't set up right now because some machines 
   * firewall HTTP traffic on the external address */
  /* find the machines IP address */
  if (consoleErr == aErrNone)
    aIO_GetInetAddr(pConsole->ioLib,
    		    &inetAddr,
    		    &consoleErr);
#endif

  /* launch the browser window */
  if ((consoleErr == aErrNone) && (pConsole->pDebugger)) {
    char addr[16];
    char url[100];
    char num[10];
    aString_FormatInetAddr(addr, inetAddr);
    aStringCopy(url, "http://");
    aStringCat(url, addr);
    aStringCat(url, ":8080/debugger?id=");
    aStringFromInt(num, (int)((module<<8) | id));
    aStringCat(url, num);

    aBrowser_LaunchURL(pConsole->uiLib, url, &consoleErr);
  }

  return consoleErr;

} /* aConsole_HandleDebug */
#endif /* aDEBUGGER */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleSP03
 */

aErr aConsole_HandleSP03(aConsole* pConsole,
		         aToken* pFirstToken,
		         aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aToken* pToken = NULL;
  char line[100];

  aVALIDCONSOLE(pConsole);

  /* first, go get the command */
  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			&pToken, NULL)) {
      consoleErr = aConsole_DisplayLine(pConsole, 
  				    "----- SP03 settings ------", 
  				    kDisplayStatus);
      aSNPRINTF(line, 100, aSP03_VOLUME, pConsole->rSP03.nVolume);
      consoleErr = aConsole_DisplayLine(pConsole, line, 
      					kDisplayStatus);
      aSNPRINTF(line, 100, aSP03_PITCH, pConsole->rSP03.nPitch);
      consoleErr = aConsole_DisplayLine(pConsole, line, 
      					kDisplayStatus);
      aSNPRINTF(line, 100, aSP03_SPEED, pConsole->rSP03.nSpeed);
      consoleErr = aConsole_DisplayLine(pConsole, line, 
      					kDisplayStatus);
      /* find the version of the module */
      if ((pConsole->nMode == modeSP03)
          && (consoleErr == aErrNone)) {
        unsigned char nChipSoftware, nChipHardware, nFirmware;
        consoleErr = aSP03Direct_GetVersion(pConsole->linkStreamRef,
        				    &nChipHardware,
        				    &nChipSoftware,
        				    &nFirmware);
        if (consoleErr == aErrNone) {
          aSNPRINTF(line, 100, "WTS701 Hardware Version: %d", 
          		       nChipHardware);
          aConsole_DisplayLine(pConsole, line, kDisplayStatus);
          aSNPRINTF(line, 100, "WTS701 Software Version: %d", 
			       nChipSoftware);
          aConsole_DisplayLine(pConsole, line, kDisplayStatus);
          aSNPRINTF(line, 100, "SP03 Firmware Version: %d", 
          		       nFirmware);
          aConsole_DisplayLine(pConsole, line, kDisplayStatus);
        } else {
          /* report the results */ 
          switch (consoleErr) {
          case aErrTimeout:
            aConsole_DisplayLine(pConsole, 
      			         aCMD_SPEECH_MISSING, 
      			         kDisplayStatus);
            break;
          default:
            aConsole_DisplayLine(pConsole, 
      			         aCMD_SPEECH_ERROR, 
      			         kDisplayStatus);
            break;
          } /* switch */
        }
      }

    } else if (pToken->eType != tkIdentifier) {
      aConsole_DisplayLine(pConsole, 
      			   aCMD_INVALID_OPTION, kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  /* see what the option was */
  if (pToken && (consoleErr == aErrNone)) {
    unsigned short val;
    if (!aStringCompare(pToken->v.identifier, "volume")) {
      consoleErr = sConsole_UShortParam(pConsole, 
    				        tokenizer, 
    				        &val,
    				        aTrue);
      if (consoleErr == aErrNone) {
        if (val > 7) {
          val = 7;
          aSNPRINTF(line, 100, aCMD_VALUE_RANGE, (int)val);
          aConsole_DisplayLine(pConsole, line, kDisplayStatus);
        }
        pConsole->rSP03.nVolume = (unsigned char)val;
      }
    } else if (!aStringCompare(pToken->v.identifier, "pitch")) {
      consoleErr = sConsole_UShortParam(pConsole, 
    				        tokenizer, 
    				        &val,
    				        aTrue);
      if (consoleErr == aErrNone) {
        if (val > 7) {
          val = 7;
          aSNPRINTF(line, 100, aCMD_VALUE_RANGE, (int)val);
          aConsole_DisplayLine(pConsole, line, kDisplayStatus);
        }
        pConsole->rSP03.nPitch = (unsigned char)val;
      }
    } else if (!aStringCompare(pToken->v.identifier, "speed")) {
      consoleErr = sConsole_UShortParam(pConsole, 
    				        tokenizer, 
    				        &val,
    				        aTrue);
      if (consoleErr == aErrNone) {
        if (val > 3) {
          val = 3;
          aSNPRINTF(line, 100, aCMD_VALUE_RANGE, (int)val);
          aConsole_DisplayLine(pConsole, line, kDisplayStatus);
        }
        pConsole->rSP03.nSpeed = (unsigned char)val;
      }
    } else if (!aStringCompare(pToken->v.identifier, "load")) {
      if (pConsole->nMode == modeSP03) {
        aStreamRef chatFile = NULL;
        char chatName[aFILE_NAMEMAXCHARS];
        consoleErr = sConsole_InputFileParam(pConsole, tokenizer,
      					     aFileAreaUser,
					     &chatFile,
      					     aTrue, chatName, 
					     ".chat");
        if (consoleErr == aErrNone) {
          consoleErr = aSP03Direct_LoadPredefines(chatFile, chatName,
        				pConsole->linkStreamRef,
					pConsole->consoleOutputRef);
        }

        if (chatFile)
          aStream_Destroy(pConsole->ioLib, chatFile, NULL);
      } else {
        aConsole_DisplayLine(pConsole, 
      			     aCMD_INVALID_MODE, 
			     kDisplayStatus);
	consoleErr = aErrParse;
      }

    } else {
      aConsole_DisplayLine(pConsole, 
      			   aCMD_INVALID_OPTION, kDisplayStatus);
      consoleErr = aErrParse;
    }

  }

  if (pToken)
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);

  return consoleErr;

} /* aConsole_HandleSP03 */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleStat
 */

aErr aConsole_HandleStat(aConsole* pConsole,
		         aToken* pFirstToken,
		         aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char line[100];
  char* linktype;
  unsigned long version;

  consoleErr = aConsole_DisplayLine(pConsole, 
  				    "---- begin statistics ----", 
  				    kDisplayStatus);

  /* the licensing information */
  if (consoleErr == aErrNone) {
    if (pConsole->bLicensed == aTrue) {
      /* build the user line */
      aStringCopySafe(line, 100, "user: ");
      aStringCatSafe(line, 100, pConsole->userFName);
      aStringCatSafe(line, 100, " ");
      aStringCatSafe(line, 100, pConsole->userLName);
      consoleErr = aConsole_DisplayLine(pConsole, line, 
      					kDisplayStatus);
      if (consoleErr == aErrNone) {
        aStringCopySafe(line, 100, "customer id: ");
        aStringCatSafe(line, 100, pConsole->userID);
        consoleErr = aConsole_DisplayLine(pConsole, line, 
        				  kDisplayStatus);
      }
      if (consoleErr == aErrNone) {
        aStringCopySafe(line, 100, "vendor: ");
        aStringCatSafe(line, 100, pConsole->vendorName);
        consoleErr = aConsole_DisplayLine(pConsole, line, 
        				  kDisplayStatus);
      }
    } else {
      aStringCopySafe(line, 100, aT_EVALUATION_MSG);
      consoleErr = aConsole_DisplayLine(pConsole, line, 
      					kDisplayStatus);
    }
  }

  if (consoleErr == aErrNone)
    consoleErr = aConsole_DisplayLine(pConsole,
      			            "-- machine information --", 
      				    kDisplayStatus);

  /* show this machine's IP address */
  if (consoleErr == aErrNone) {
    aErr ipErr;
    unsigned long inetAddr;
    char addr[16];
    aStringCopySafe(line, 100, "TCP/IP ");
    aIO_GetInetAddr(pConsole->ioLib, &inetAddr, &ipErr);
    if (ipErr == aErrNone) {
      aStringCatSafe(line, 100, "address: ");
      aUtil_FormatInetAddr(addr, inetAddr, NULL);
      aStringCatSafe(line, 100, addr);
    } else {
      aStringCatSafe(line, 100, "not supported");
    }
    consoleErr = aConsole_DisplayLine(pConsole,
      			              line, 
      				      kDisplayStatus);
  }

  /* show the link type being used */
  if (consoleErr == aErrNone) {
    aSettingFile_GetString(pConsole->ioLib, 
    			   pConsole->settings, 
    			   LINKTYPEKEY, 
    			   &linktype,
     			   DEFAULTLINKTYPE, 
     			   &consoleErr);
    if (consoleErr == aErrNone) {
      aStringCopySafe(line, 100, aSTATUS_LINKTYPE);
      aStringCatSafe(line, 100, linktype);
      consoleErr = aConsole_DisplayLine(pConsole, line, 
      				        kDisplayStatus);
    }
  }

  if (consoleErr == aErrNone) {
    if (!aStringCompare(linktype, "serial")) {
      /* show the portname being used */
      if (consoleErr == aErrNone) {
	char* portname;
	
	aSettingFile_GetString(pConsole->ioLib, 
			       pConsole->settings, 
			       PORTNAMEKEY, 
			       &portname,
			       DEFAULTPORTNAME, 
			       &consoleErr);
	if (consoleErr == aErrNone) {
	  aStringCopySafe(line, 100, aSTATUS_PORTNAME);
	  aStringCatSafe(line, 100, portname);
	  consoleErr = aConsole_DisplayLine(pConsole, line, 
					    kDisplayStatus);
	}
      }
      
      /* show the baudrate being used */
      if (consoleErr == aErrNone) {
	int baudrate;
	aSettingFile_GetInt(pConsole->ioLib, 
			    pConsole->settings, 
			    BAUDRATEKEY, 
			    &baudrate,
			    DEFAULTBAUDRATE, 
			    &consoleErr);
	if (consoleErr == aErrNone) {
	  aSNPRINTF(line, 100, aSTATUS_BAUDRATE, baudrate);
	  consoleErr = aConsole_DisplayLine(pConsole, line, 
					    kDisplayStatus);
	}
      }
    } else if (!aStringCompare(linktype, "usb")) {

      /* show the usb serial number being */
      if (consoleErr == aErrNone) {
	int serialNum;
	aSettingFile_GetInt(pConsole->ioLib, 
			    pConsole->settings, 
			    USBSERIALKEY, 
			    &serialNum,
			    0, 
			    &consoleErr);
	if (consoleErr == aErrNone) {
	  aSNPRINTF(line, 100, aSTATUS_USBID, serialNum);
	  consoleErr = aConsole_DisplayLine(pConsole, line, 
					    kDisplayStatus);
	}
      }
    }
  }

  if (consoleErr == aErrNone)
    consoleErr = aConsole_DisplayLine(pConsole,
      			            "--- software versions  ---", 
      				    kDisplayStatus);

  /* console version information */
  if (consoleErr == aErrNone)
    consoleErr = aConsole_ShowVersion(pConsole, "Console", 
                                      (aVERSION_MAJOR << 28) |
                                      (aVERSION_MINOR << 24) |
                                      aCONSOLE_BUILD_NUM);

  /* library version numbers */
  if ((consoleErr == aErrNone)
      && !aIO_GetVersion(pConsole->ioLib, &version, &consoleErr))
    consoleErr = aConsole_ShowVersion(pConsole, "aIO", version);
  if ((consoleErr == aErrNone)
      && !aUI_GetVersion(pConsole->uiLib, &version, &consoleErr))
    consoleErr = aConsole_ShowVersion(pConsole, "aUI", version);
  if ((consoleErr == aErrNone)
      && !aStem_GetVersion(pConsole->stemLib, &version, &consoleErr))
    consoleErr = aConsole_ShowVersion(pConsole, "aStem", version);
  if ((consoleErr == aErrNone)
      && !aSteep_GetVersion(pConsole->steepLib, &version, &consoleErr))
    consoleErr = aConsole_ShowVersion(pConsole, "aSteep", version);
  if ((consoleErr == aErrNone)
      && !aLeaf_GetVersion(pConsole->leafLib, &version, &consoleErr))
    consoleErr = aConsole_ShowVersion(pConsole, "aLeaf", version);
  if ((consoleErr == aErrNone)
      && !aTEAvm_GetVersion(pConsole->vmLib, &version, &consoleErr))
    consoleErr = aConsole_ShowVersion(pConsole, "aTEAvm", version);

  /* line feed */
  consoleErr = aConsole_DisplayLine(pConsole,
      			            "------ console mode ------", 
      				    kDisplayStatus);

  /* show the mode we are running in */
  if (consoleErr == aErrNone) {
    switch (pConsole->nMode) {

    case modeSP03:
      aConsole_DisplayLine(pConsole, aSTATUS_SP03_MODE, 
      			   kDisplayStatus);
      break;

    case modeBrainStem:
      consoleErr = aConsole_DisplayLine(pConsole, 
      					aSTATUS_BRAINSTEM_MODE, 
      					kDisplayStatus);
      break;
    }
  }

  consoleErr = aConsole_DisplayLine(pConsole, 
  				    "----- end statistics -----", 
  				    kDisplayStatus);

  return consoleErr;

} /* aConsole_HandleStat */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleSteep
 */

aErr aConsole_HandleSteep(aConsole* pConsole,
			  int compileFlags,
		          aToken* pFirstToken,
		          aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char filename[aMAXIDENTIFIERLEN];
  char destfile[aFILE_NAMEMAXCHARS];
  aFileArea resultArea;
  aBool bCompiled = aFalse;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone)
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaUser,
    					 NULL,
    					 aTrue,
    					 filename,
    					 ".tea");

  /* build the output file name */
  if (consoleErr == aErrNone) {
    aString_GetFileRoot(destfile, filename);
    resultArea = aFileAreaUser;
    if (aStringCompare(destfile, "") == 0) {
      aStringCopySafe(destfile, aFILE_NAMEMAXCHARS, filename);
    }
    if (compileFlags & fSteepSAST)
      aStringCatSafe(destfile, aFILE_NAMEMAXCHARS, ".ast");
    else if (compileFlags & fSteepPreprocess)
      aStringCatSafe(destfile, aFILE_NAMEMAXCHARS, ".pp");
    else {
      aStringCatSafe(destfile, aFILE_NAMEMAXCHARS, ".cup");
      resultArea = aFileAreaObject;
    }
  }

  if (consoleErr == aErrNone) {
    consoleErr = aConsole_Compile(pConsole, compileFlags,
    				  pConsole->consoleOutputRef,
    				  pConsole->consoleOutputRef,
    				  filename,
    				  aFileAreaUser,
    				  destfile,
    				  resultArea);
    if (consoleErr == aErrNotFound) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_FILE_NOT_FOUND, kDisplayStatus);
    } else if (consoleErr == aErrIO) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_FILE_IO_ERROR, kDisplayStatus);
    } else if (consoleErr != aErrNone) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_FILE_ERROR, kDisplayStatus);
    } else {
      bCompiled = aTrue;
    }
  }

  /* report the resulting code size */
  if ((consoleErr == aErrNone) && bCompiled) {
    aErr err;
    aFileRef result;
    unsigned long resultSize = 0;
    char line[100];
    aFile_Open(pConsole->ioLib,
    	       destfile, 
    	       aFileModeReadOnly,
    	       resultArea,
    	       &result,
    	       &err);
    if (err == aErrNone)
      aFile_GetSize(pConsole->ioLib, result, 
      		    &resultSize, &err);
    if (err == aErrNone)
      aFile_Close(pConsole->ioLib, result, &err);
    if ((err == aErrNone) && resultSize) {
      aSNPRINTF(line, 100, aCMD_COMPILE_SIZE, (int)resultSize);
      aConsole_DisplayLine(pConsole, line, kDisplayStatus);
    }
  }

  return consoleErr;

} /* aConsole_HandleSteep */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleLeaf
 */

aErr aConsole_HandleLeaf(aConsole* pConsole,
			 int compileFlags,
		         aToken* pFirstToken,
		         aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char filename[aMAXIDENTIFIERLEN];
  char destfile[aFILE_NAMEMAXCHARS];
  aFileArea resultArea;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone)
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaUser,
    					 NULL,
    					 aTrue,
    					 filename,
					 ".leaf");

  /* build the output file name */
  if (consoleErr == aErrNone) {
    aString_GetFileRoot(destfile, filename);
    resultArea = aFileAreaUser;
    if (aStringCompare(destfile, "") == 0) {
      aStringCopySafe(destfile, aFILE_NAMEMAXCHARS, filename);
    }
    if (compileFlags & fLeafAST)
      aStringCatSafe(destfile, aFILE_NAMEMAXCHARS, ".last");
    else if (compileFlags & fLeafPreprocess)
      aStringCatSafe(destfile, aFILE_NAMEMAXCHARS, ".pp");
    else {
      aStringCatSafe(destfile, aFILE_NAMEMAXCHARS, ".bag");
    }
  }

  if (consoleErr == aErrNone) {
    consoleErr = aConsole_Leaf(pConsole, 
    			       compileFlags,
    			       pConsole->consoleOutputRef,
    			       pConsole->consoleOutputRef,
    			       filename,
    			       aFileAreaUser,
    			       destfile,
    			       resultArea);
    if (consoleErr == aErrNotFound) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_FILE_NOT_FOUND, kDisplayStatus);
    } else if (consoleErr == aErrIO) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_FILE_IO_ERROR, kDisplayStatus);
    } else if (consoleErr != aErrNone) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_FILE_ERROR, kDisplayStatus);
    }
  }

  return consoleErr;

} /* aConsole_HandleLeaf */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleRelay
 */

aErr aConsole_HandleRelay(aConsole* pConsole,
		          aToken* pFirstToken,
		          aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  unsigned long inetAddr;
  unsigned short portNum;

  /* we don't do this in other modes */
  if ((consoleErr == aErrNone)
      && (pConsole->nMode != modeBrainStem)) {
    aConsole_DisplayLine(pConsole, 
    			 aCMD_INVALID_MODE, 
    			 kDisplayStatus);
    consoleErr = aErrParse;
  }

  /* find the machines IP address */
  if (consoleErr == aErrNone)
    aIO_GetInetAddr(pConsole->ioLib,
    		    &inetAddr,
    		    &consoleErr);

  if (consoleErr == aErrNone)
    consoleErr = sConsole_UShortParam(pConsole, 
    				      tokenizer, 
    				      &portNum,
    				      aTrue);
  if (consoleErr == aErrNone)
    aStream_CreateSocket(pConsole->ioLib,
    			 inetAddr,
    			 portNum,
    			 aTrue,
    			 &pConsole->relayStreamRef,
    			 &consoleErr);

  if (consoleErr == aErrNone)
    aStem_SetStream(pConsole->stemLib,
    		    pConsole->relayStreamRef,
    		    kStemRelayStream,
    		    &consoleErr);
  
  if (consoleErr == aErrNone) {
    char addr[16];
    char line[100];
    aUtil_FormatInetAddr(addr, inetAddr, NULL);
    aStringCopySafe(line, 100, aRLY_STARTING);
    aStringCatSafe(line, 100, addr);
    aConsole_DisplayLine(pConsole, line, kDisplayStatus);
  }

  return consoleErr;

} /* aConsole_HandleRelay */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleRaw
 */

aErr aConsole_HandleRaw(aConsole* pConsole,
		        aToken* pFirstToken,
		        aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char data[MAXINPUTLINE];
  unsigned char dataLen;

  if (pConsole->linkStreamRef != NULL) {
  
    /* get the raw data */
    consoleErr = aConsole_ParseRaw(pConsole, 
    				   tokenizer,
    				   data,
    				   &dataLen,
    				   MAXINPUTLINE);
    				 
    if (consoleErr == aErrNone) {
      aStream_Write(pConsole->ioLib,
      		    pConsole->linkStreamRef,
      		    data,
      		    dataLen,
      		    &consoleErr);
    }

  } else {
    consoleErr = aConsole_DisplayLine(pConsole, aSTATUS_NO_LINK, 
    				      kDisplayStatus);
  }

  return consoleErr;

} /* aConsole_HandleRaw */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleSay
 */

aErr aConsole_HandleSay(aConsole* pConsole,
		        aToken* pFirstToken,
		        aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aErr speakErr = aErrNone;
  aToken* pToken = NULL;
  unsigned char nPhrase;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone)  {
    if (aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			&pToken, NULL)) {
      aConsole_DisplayLine(pConsole, 
      			   aCMD_MISSING_PARAM, kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  if (consoleErr == aErrNone) {

    /* here we have a token but don't know its type */
    switch (pToken->eType) {

    case tkInt:
      nPhrase = (unsigned char)pToken->v.integer;
      if ((nPhrase == 0) || (nPhrase > aSP03_MAXNUMPHRASES)) {
        aConsole_DisplayLine(pConsole, 
      			     aCMD_SPEECH_RANGE_ERROR, 
      			     kDisplayStatus);
        consoleErr = aErrParse;
      } else {

        switch(pConsole->nMode) {

        case modeSP03:
          speakErr = aSP03Direct_SpeakPhrase(pConsole->linkStreamRef,
        			             nPhrase);
          break;

        case modeBrainStem:
          speakErr = aSP03_SpeakPhrase(pConsole->stemLib,
        			       nPhrase);
          break;
        
        default:
          speakErr = aErrConfiguration;
          break;

        } /* switch */
      }
      break;

    case tkString:
      switch(pConsole->nMode) {

      case modeSP03:
        speakErr = aSP03Direct_SpeakString(pConsole->linkStreamRef,
        				   pConsole->rSP03.nVolume, 
        				   pConsole->rSP03.nPitch, 
        				   pConsole->rSP03.nSpeed,
      				           pToken->v.string);
        break;

      case modeBrainStem:
        speakErr = aSP03_SpeakString(pConsole->stemLib,
        			     pConsole->rSP03.nVolume, 
        			     pConsole->rSP03.nPitch, 
        			     pConsole->rSP03.nSpeed,
        			     pToken->v.string);
        break;

      default:
        speakErr = aErrConfiguration;
        break;
      } /* switch */
      break;

    default:
      aConsole_DisplayLine(pConsole, 
      			   aCMD_ILLEGAL_INPUT, kDisplayStatus);
      consoleErr = aErrParse;
      break;
    }

    /* report the results */ 
    if (consoleErr == aErrNone) {
      switch (speakErr) {
      case aErrNone:
        aConsole_DisplayLine(pConsole, 
      			     aCMD_SPEECH_DONE, kDisplayStatus);
        break;
      case aErrTimeout:
        aConsole_DisplayLine(pConsole, 
      			     aCMD_SPEECH_MISSING, kDisplayStatus);
        break;
      case aErrConfiguration:
        aConsole_DisplayLine(pConsole, 
      			     aCMD_INVALID_MODE, kDisplayStatus);
        break;
      default:
        aConsole_DisplayLine(pConsole, 
      			     aCMD_SPEECH_ERROR, kDisplayStatus);
        break;
      } /* switch */
    }

    /* we are done with the token so release it */
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  if (consoleErr == aErrNone) {

    /* here we have a port reference, check for extra input */
    if (!aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			 &pToken, NULL)) {
      aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
      pToken = NULL;
      aConsole_DisplayLine(pConsole, aCMD_UNEXPECTED_PARAM, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  /* clean up on error */
  if (pToken != NULL)
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);

  return consoleErr;  

} /* aConsole_HandleSay */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleDump
 */

aErr aConsole_HandleDump(aConsole* pConsole,
		         aToken* pFirstToken,
		         aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char file[aMAXIDENTIFIERLEN];
  aStreamRef fileStream = NULL;
  aStreamRef dumpStream = NULL;
  int i;
  char data;

  aVALIDCONSOLE(pConsole);

  /* dump takes a single filename parameter */
  if (consoleErr == aErrNone)
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaUser, 
    					 &fileStream,
    					 aFalse,
    					 file,
    					 NULL);

  if (consoleErr == aErrNone)
    aStream_CreateFileOutput(pConsole->ioLib,
    			     "dump",
    			     aFileAreaUser,
    			     &dumpStream,
    			     &consoleErr);

  if (consoleErr == aErrNone) {
    char line[100];
    char num[10];
    i = 0;
    while(consoleErr == aErrNone) {
      if (!aStream_Read(pConsole->ioLib, 
    			fileStream, &data,
    			1, &consoleErr)) {
        aStringFromInt(num, i++);
        aStringCopySafe(line, 100, num);
        aStringCatSafe(line, 100, ": ");
        aStringFromInt(num, data);
        aStringCatSafe(line, 100, num);
        aStringCatSafe(line, 100, ", ");
        num[0] = '[';
        num[1] = data;
        num[2] = ']';
        num[3] = 0;
        aStringCatSafe(line, 100, num);
        aStream_WriteLine(pConsole->ioLib, dumpStream,
        		  line, &consoleErr);
      }
    }
  }

  if (fileStream != NULL)
    aStream_Destroy(pConsole->ioLib, fileStream, NULL);
  if (dumpStream != NULL)
    aStream_Destroy(pConsole->ioLib, dumpStream, NULL);
  
  return consoleErr;

} /* aConsole_HandleDump */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleHelp
 */

aErr aConsole_HandleHelp(aConsole* pConsole,
		         aToken* pFirstToken,
			 aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  char param[aMAXIDENTIFIERLEN];

  aVALIDCONSOLE(pConsole);

  /* see if anything followed the help command */
  if (consoleErr == aErrNone)  {
    aToken* pToken = NULL;
    aErr tokenErr;
    param[0] = 0;
    if (!aTokenizer_Next(pConsole->ioLib, 
    			 tokenizer, &pToken, &tokenErr)) {
    
      /* make sure it was a string */
      aAssert(pToken);
      if (pToken->eType != tkIdentifier) {
        aConsole_DisplayLine(pConsole, 
        		     aCMD_HELP_CMD_EXPECTED, 
        		     kDisplayStatus);
      } else {
        aStringCopySafe(param, aMAXIDENTIFIERLEN, pToken->v.identifier);
      }
      aTokenizer_Dispose(pConsole->ioLib, 
      			 tokenizer, pToken, &consoleErr);
    }
  }

  /* now param contains the string or is "" */

  /* do the help */
  if (consoleErr == aErrNone) {

    /* no param shows general command list */
    if (aStringCompare(param, "") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   "commands:", 
      			   kDisplayStatus);

      aConsole_DisplayLine(pConsole, 
      			   " ast           leaf", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " batch         load", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " dsm           pp", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " dump          raw", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " exit          relay", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " help          say",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " SP03          stat", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " last          steep", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " launch        unload", 
      			   kDisplayStatus);

      aConsole_DisplayLine(pConsole, 
      			   "type \"help <command>\" for specifics", 
      			   kDisplayStatus);    

    /* ast command */
    } else if (aStringCompare(param, "ast") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " ast <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates annotated syntax tree of TEA file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates AST file in aUser", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     .tea filename extension is optional", 
      			   kDisplayStatus);

    /* batch command */
    } else if (aStringCompare(param, "batch") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " batch <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   issues commands stored in a file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     filename extension must be supplied", 
      			   kDisplayStatus);

    /* dsm command */
    } else if (aStringCompare(param, "dsm") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " dsm <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   decompiles a CUP file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates DSM file in aUser", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     .cup filename extension is optional", 
      			   kDisplayStatus);

    /* dump command */
    } else if (aStringCompare(param, "dump") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " dump <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   converts file in aUser to hex text", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   writes file named 'dump' to aUser", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     filename extension must be supplied", 
      			   kDisplayStatus);

    /* exit command */
    } else if (aStringCompare(param, "exit") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " exit", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   terminates Console app", 
      			   kDisplayStatus);

    /* help command */
    } else if (aStringCompare(param, "help") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " help", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   shows list of Console commands", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " help <command name>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   shows usage of particular command", 
      			   kDisplayStatus);

    /* last command */
    } else if (aStringCompare(param, "last") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " last <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates annotated syntax tree of LEAF file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates LAST file in aUser", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     .leaf extension is optional", 
      			   kDisplayStatus);

    /* launch command */
    } else if (aStringCompare(param, "launch") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " launch <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   executes TEA file in Console emulator", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " launch <module ID> <file slot>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   executes TEA file in module", 
      			   kDisplayStatus);

    /* leaf command */
    } else if (aStringCompare(param, "leaf") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " leaf <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   compiles a LEAF file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates BAG file in aUser", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     .leaf extension is optional", 
      			   kDisplayStatus);

    /* load command */
    } else if (aStringCompare(param, "load") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " load <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   downloads CUP file to module", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   CUP file must be created with steep", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     .cup extension is optional", 
      			   kDisplayStatus);

    /* pp command */
    } else if (aStringCompare(param, "pp") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " pp <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   preprocess a TEA file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates PP file in aUser", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     .tea extension is optional", 
      			   kDisplayStatus);

    /* raw command */
    } else if (aStringCompare(param, "raw") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " raw <sequence of numbers>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   sends sequence of bytes to module", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   does not enforce packet protocol", 
      			   kDisplayStatus);

    /* relay command */
    } else if (aStringCompare(param, "relay") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " relay <IP address> <TCP/IP port>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   configures Console as TCP/IP relay", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   packets will be sent to TCP/IP host", 
      			   kDisplayStatus);

    /* stat command */
    } else if (aStringCompare(param, "stat") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " stat", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   displays current software version info", 
      			   kDisplayStatus);

    /* say command */
    } else if (aStringCompare(param, "say") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " say <phrase num> or say <quoted string>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   speaks the requested phrase or text", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   using the SP03 speech module", 
      			   kDisplayStatus);

    /* SP03 command */
    } else if (aStringCompare(param, "SP03") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " SP03", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   shows the current SP03 info", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " SP03 volume <number>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   sets the current volume", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     range (0, loud - 7, quiet)", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " SP03 pitch <number>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   sets the current pitch", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     range (0 high - 7 low)", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " SP03 speed <number>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   sets the current speed", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     range (0 slow - 3 fast)", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   " SP03 load <chat file>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   loads the specified phrase file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   defaults to \".chat\" extension", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   only available in SP03 mode", 
      			   kDisplayStatus);

    /* steep command */
    } else if (aStringCompare(param, "steep") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " steep <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   compiles a TEA file", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   creates CUP file in aObject", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     .tea extension is optional", 
      			   kDisplayStatus);

    /* unload command */
    } else if (aStringCompare(param, "unload") == 0) {
      aConsole_DisplayLine(pConsole, 
      			   " unload <module ID> <file slot> <filename>", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   reads file from module and", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   saves it as binary file in aObject", 
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "   note: ",
      			   kDisplayStatus);
      aConsole_DisplayLine(pConsole, 
      			   "     filename extension must be supplied", 
      			   kDisplayStatus);


    } else {
      aConsole_DisplayLine(pConsole, 
        	           aCMD_HELP_CMD_EXPECTED, 
        	           kDisplayStatus);
    }

    /* add a line at the bottom */
    aConsole_DisplayLine(pConsole, "", kDisplayStatus);    
  }

  return consoleErr;
  
} /* aConsole_HandleHelp */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleMakeDump
 */

aErr aConsole_HandleMakeDump(aConsole* pConsole, 
			     aToken* pFirstToken, 
			     aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aStreamRef fileStream = NULL;
  aStreamRef dumpStream = NULL;
  char filename[aMAXIDENTIFIERLEN];
  char destfile[aMAXIDENTIFIERLEN];
  aBDRef bd;

  aVALIDCONSOLE(pConsole);

  /* first, get the dump description file name */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaUser,
    					 &fileStream,
    					 aFalse,
    					 filename,
    					 ".xml");
  }
 
  if (consoleErr == aErrNone)
    consoleErr = aBD_Create(pConsole->ioLib, pConsole->stemLib, NULL, NULL, &bd);
  
  if (consoleErr == aErrNone)
    consoleErr = aBD_ReadXML(bd, fileStream, 
    			     pConsole->consoleOutputRef);


  /* build the output file name */
  if (consoleErr == aErrNone) {
    aString_GetFileRoot(destfile, filename);
    if (aStringCompare(destfile, "") == 0) {
      aStringCopySafe(destfile, aMAXIDENTIFIERLEN, filename);
    }
    aStringCatSafe(destfile, aMAXIDENTIFIERLEN, ".dump");
  }

  /* open the output file */
  if (consoleErr == aErrNone)    
    aStream_CreateFileOutput(pConsole->ioLib,
    			     destfile,
    			     aFileAreaObject,
    			     &dumpStream,
    			     &consoleErr);    			     

  if (consoleErr == aErrNone)
    consoleErr = aBD_Write(bd, dumpStream);

  if (consoleErr == aErrNone)
    aStream_Destroy(pConsole->ioLib, dumpStream, &consoleErr);

  if (consoleErr == aErrNone)
    consoleErr = aBD_Destroy(bd);

  /* clean up the file either way if it was opened */
  if (fileStream != NULL) {
    aStream_Destroy(aStreamLibRef(fileStream), fileStream, NULL);
    fileStream = NULL;
  }
  
  return consoleErr;

} /* aConsole_HandleMakeDump */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleBrainDump
 */

aErr aConsole_HandleBrainDump(aConsole* pConsole, 
			      aToken* pFirstToken, 
			      aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aStreamRef fileStream = NULL;
  char filename[aMAXIDENTIFIERLEN];
  aBDRef bd;
  aBool bCreated = aFalse;

  aVALIDCONSOLE(pConsole);

  /* first, get the dump file name */
  if (consoleErr == aErrNone) {
    consoleErr = sConsole_InputFileParam(pConsole, 
    					 tokenizer, 
    					 aFileAreaObject,
    					 &fileStream,
    					 aFalse,
    					 filename,
    					 ".dump");
  }
 
  if (consoleErr == aErrNone)
    consoleErr = aBD_Create(pConsole->ioLib, pConsole->stemLib, sShowStatus, pConsole, &bd);
  
  if (consoleErr == aErrNone) {
    bCreated = aTrue;
    consoleErr = aBD_Read(bd, fileStream);
  }

  if (consoleErr == aErrNone)
    consoleErr = aBD_Dump(bd);

  if (consoleErr != aErrNone) {
    if (consoleErr == aErrConnection) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_NO_MODULE_FOUND, kDisplayStatus);
    } else if (consoleErr == aErrConfiguration) {
      consoleErr = aConsole_DisplayLine(pConsole, 
      			aCMD_MODULE_DOES_NOT_REPLY, kDisplayStatus);
    }
  }

  /* clean up dump if it has been created */
  if (bCreated)
    consoleErr = aBD_Destroy(bd);

  /* clean up the file either way if it was opened */
  if (fileStream != NULL) {
    aStream_Destroy(aStreamLibRef(fileStream), fileStream, NULL);
    fileStream = NULL;
  }

  consoleErr = aConsole_DisplayLine(pConsole, "done", kDisplayStatus);
  
  return consoleErr;

} /* aConsole_HandleBrainDump */
