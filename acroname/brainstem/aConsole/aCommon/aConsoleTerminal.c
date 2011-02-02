/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsoleTerminal.c			 		   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of platform-independent Terminal    */
/*              mode handling.					   */
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

int foo;

#ifdef aTERMINAL

#include "aUtil.h"
#include "aConsole.h"
#include "aConsoleTerminal.h"
#include "aStream_TextLine.h"
#include "aConsoleText.h"
#include "aConsole_Cmds.h"


/* special character used for file transfer */
#define SOH          1          /* Start Of Header */
#define EOT          4          /* End Of Transmission */
#define ACK          6          /* Acknowledge (positive) */
#define DLE          16         /* Data Link Escape */
#define XON          17         /* Transmit On */
#define XOFF         19         /* Transmit Off */
#define NAK          21         /* Negative Acknowledge */
#define SYN          22         /* Synchronous idle */
#define CAN          24         /* Cancel */
#ifndef EOF
#define EOF	     26
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines
 */

typedef struct aXMODEM {
  aIOLib		ioRef;
  aConsole* 		pConsole;
  unsigned char 	data[132];	/* packet storage */
  unsigned char 	nSequence;
  aFileRef		fileRef;
  unsigned long		nSize;
  unsigned long		nWritten;
  aStreamRef		stream;
} aXMODEM;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines
 */

static aStreamRef debug_log = NULL;

static aErr sXMODEM_Init(aXMODEM* pXMODEM,
			 aConsole* pConsole,
			 aStreamRef stream,
			 aFileRef fileRef);
static aErr sXMODEM_Next(aXMODEM* pXMODEM);
static aErr sXMODEM_Write(aXMODEM* pXMODEM);
static aErr sXMODEM_Close(aXMODEM* pXMODEM);

static aErr sXMODEM_GetResponse(aConsole* pConsole, 
			        const unsigned char desired,
		                const unsigned long nMSTimeout);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleTerminal_TimeSlice
 */

aErr aConsoleTerminal_TimeSlice(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  aErr readErr;
  char c;

  /* if we are in some processing mode, just return */
  if (pConsole->nTerminalFlags)
    return consoleErr;

  /* swallow up any available characters and send them to the 
   * display buffer */
  do {
    aStream_Read(pConsole->ioLib, 
    		 pConsole->linkStreamRef,
    		 &c, 1, &readErr);
    switch (readErr) {

    case aErrNone:
      aStream_Write(pConsole->ioLib, pConsole->consoleMessageRef,
      		    &c, 1, &readErr);
      break;

    case aErrNotReady:
      readErr = aErrNotFound;
      break;
    
    case aErrIO:
      break;

    default:
      consoleErr = readErr;
      break;

    } /* switch */

  } while (readErr == aErrNone);

  return consoleErr;

} /* aConsoleTerminal_TimeSlice */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleTerminal_HandleLine
 */

aErr aConsoleTerminal_HandleLine(aConsole* pConsole,
				 const char* pLine)
{
  aErr consoleErr = aErrNone;
  aBool bCommand = aFalse;


  /* check for commands */
  if (consoleErr == aErrNone) {
    const char* pRest;
    aStreamRef commandStream;
    pRest = aString_StartsWith(pLine, aCONSOLETERMCMD);
    if (pRest) {

      bCommand = aTrue;
      
      /* create a steam for use in tokenizing the command line */
      consoleErr = aStream_Create_TextLine_Input(pConsole->ioLib, 
    					         pRest, 
    					         &commandStream);
      if (consoleErr == aErrNone)
        aConsoleTerminal_HandleCommand(pConsole, commandStream);

    /* if not a console command, just send it along the line */
    } else {
      aStream_Write(pConsole->ioLib, 
    		    pConsole->linkStreamRef,
    		    pLine, aStringLen(pLine),
    		    &consoleErr);
      if (consoleErr == aErrNone)
        aStream_Write(pConsole->ioLib, 
    		      pConsole->linkStreamRef,
    		      "\n", 1,
    		      &consoleErr);
    }
  }

  return consoleErr;

} /* aConsoleTerminal_HandleLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleTerminal_HandleCommand
 *
 * this routine cleans up the passed in commandStream when finished
 */

aErr aConsoleTerminal_HandleCommand(aConsole* pConsole,
				    aStreamRef commandStream)
{
  aErr consoleErr = aErrNone;
  aTokenizerRef tokenizer;
  aToken* pToken;
  aBool bAvail = aFalse;

  aVALIDCONSOLE(pConsole);

  if (consoleErr == aErrNone)
    aTokenizer_Create(pConsole->ioLib, 
    		      commandStream, 
    		      aT_CMDLINENAME,
    		      aFileAreaUser,
    		      NULL,
    		      NULL, 
    		      &tokenizer, 
    		      &consoleErr);

  /* inspect the first token to see if it is a command */
  if (consoleErr == aErrNone)
    bAvail = !aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			      &pToken, NULL);

  if (bAvail == aTrue) {
    switch (pToken->eType) {

    case tkIdentifier:
      consoleErr = aConsoleTerminal_ParseCommand(pConsole,
      					         pToken,
      					         tokenizer);
      break;

    default:
      aConsole_DisplayLine(pConsole, aCMD_ILLEGAL_INPUT, 
      			   kDisplayStatus);
      break;

    } /* switch */

    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, 
    		       &consoleErr);
  }

  if (consoleErr == aErrNone)
    aTokenizer_Destroy(pConsole->ioLib, tokenizer, &consoleErr);

  return consoleErr;

} /* aConsoleTerminal_HandleCommand */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleTerminal_ParseCommand
 */

aErr aConsoleTerminal_ParseCommand(aConsole* pConsole,
			           aToken* pCommandToken,
			           aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;

  aAssert(pCommandToken->eType == tkIdentifier);

  if (!aStringCompare(pCommandToken->v.identifier, "exit")) {
    aConsole_HandleExit(pConsole, pCommandToken, tokenizer);
  } else if (!aStringCompare(pCommandToken->v.identifier, 
  			     "xmodem")) {
    aConsoleTerminal_XMODEM(pConsole, pCommandToken, tokenizer);
  }

  return consoleErr;

} /* end of aConsoleTerminal_ParseCommand */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleTerminal_XMODEM
 */

aErr aConsoleTerminal_XMODEM(aConsole* pConsole,
			     aToken* pFirstToken,
			     aTokenizerRef tokenizer)
{
  aErr consoleErr = aErrNone;
  aToken* pToken = NULL;
  aToken* pS1 = NULL;
  aToken* pS2 = NULL;
  int mode = 0;

  aVALIDCONSOLE(pConsole);

  if ((consoleErr == aErrNone) 
      && aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			 &pToken, NULL)) {
    aConsole_DisplayLine(pConsole, aCMD_HELP_CMD_EXPECTED, 
      			 kDisplayStatus);
    consoleErr = aErrParse;
  }

  if (pToken && (consoleErr == aErrNone)) {
    if (pToken->eType == tkIdentifier) {
      if (!aStringCompare(pToken->v.identifier, "send"))
        mode = 1;
      else if (!aStringCompare(pToken->v.identifier, "receive"))
        mode = 2;
    }
    if (!mode) {
      aConsole_DisplayLine(pConsole, aCMD_INVALID_OPTION, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  /* clean up the command token either way */
  if (pToken) {
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pToken, NULL);
    pToken = NULL;
  }

  /* now, look for the first string which is the command for 
   * the other side */
  if ((consoleErr == aErrNone) 
      && aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			 &pS1, NULL)) {
    aConsole_DisplayLine(pConsole, aCMD_EXPECTED_STRING, 
      			 kDisplayStatus);
    consoleErr = aErrParse;
  }
  if (pS1 && (consoleErr == aErrNone)) {
    if (pS1->eType != tkString) {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_STRING, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }

  /* finally, the filename from this side (either source or 
   * dest) */
  if ((consoleErr == aErrNone) 
      && aTokenizer_Next(pConsole->ioLib, tokenizer, 
    			 &pS2, NULL)) {
    aConsole_DisplayLine(pConsole, aCMD_EXPECTED_STRING, 
      			 kDisplayStatus);
    consoleErr = aErrParse;
  }
  if (pS2 && (consoleErr == aErrNone)) {
    if (pS2->eType != tkString) {
      aConsole_DisplayLine(pConsole, aCMD_EXPECTED_STRING, 
      			   kDisplayStatus);
      consoleErr = aErrParse;
    }
  }
  
  /* if we make it here clean, we can actually do the transfer */
  if (consoleErr == aErrNone) {
    switch (mode) {
    case 1: /* send */
      consoleErr = aConsoleTerminal_XMODEM_Send(pConsole,
      						pS1->v.string,
      						pS2->v.string);
      break;
    case 2: /* receive */
      break;
    } /* switch */
  }

  /* clean up the command token either way */
  if (pS1) {
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pS1, NULL);
    pS1 = NULL;
  }
  if (pS2) {
    aTokenizer_Dispose(pConsole->ioLib, tokenizer, pS2, NULL);
    pS2 = NULL;
  }

  return consoleErr;

} /* aConsoleTerminal_XMODEM */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsoleTerminal_XMODEM_Send
 */

aErr aConsoleTerminal_XMODEM_Send(aConsole* pConsole,
			          const char* pCommand,
			          const char* pFilename)
{
  aErr consoleErr = aErrNone;
  aFileRef file = NULL;
  unsigned long written = 0;
  unsigned char sequence = 1;
  char num[20];
  float percent = 0.0f;
  aXMODEM xmodem;
  int packet;

  aVALIDCONSOLE(pConsole);

  aStream_CreateFileOutput(pConsole->ioLib,
  			   "xmodem.Log",
  			   aFileAreaUser,
  			   &debug_log, NULL);

  /* try to find and open the file to be sent */
  if (consoleErr == aErrNone)
    aFile_Open(pConsole->ioLib, pFilename, 
    	       aFileModeReadOnly,
      	       aFileAreaUser, &file, &consoleErr);

  /* show an error if the file isn't found */
  if (consoleErr == aErrNotFound)
    aConsole_DisplayLine(pConsole, aCMD_FILE_INVALID, 
    			 kDisplayStatus);

  /* initialize the XMODEM transfer object */
  if (consoleErr == aErrNone)
    consoleErr = sXMODEM_Init(&xmodem, 
    			      pConsole,
    			      pConsole->linkStreamRef,
    			      file);

  /* tell the user we are transfering */
  if (consoleErr == aErrNone) {
    aStringFromInt(num, xmodem.nSize);
    aConsole_BufferOutput(pConsole, "sending \"");
    aConsole_BufferOutput(pConsole, pFilename);
    aConsole_BufferOutput(pConsole, "\" (");
    aConsole_BufferOutput(pConsole, num);
    aConsole_BufferOutput(pConsole, " bytes) using xmodem");
    aConsole_DisplayBuffer(pConsole, kDisplayStatus);
  }

  /* "type" the transfer command on the remote */
  if (consoleErr == aErrNone)
    consoleErr = aConsoleTerminal_HandleLine(pConsole, pCommand);

  /* now, look for the response NAK, waiting for 20 seconds */
  if (consoleErr == aErrNone)
    consoleErr = sXMODEM_GetResponse(pConsole, NAK, 20000);

  /* now send the file in pieces */
  packet = 0;
  while (consoleErr == aErrNone) {
    int i;

    /* build up the next packet */
    consoleErr = sXMODEM_Next(&xmodem);

    /* clean up if we are done */
    if (consoleErr == aErrEOF)
      consoleErr = sXMODEM_Close(&xmodem);

    /* try to send the packet up to 10 times */
    i = 0;
    while ((consoleErr == aErrNone) && (i < 10)) {
      consoleErr = sXMODEM_Write(&xmodem);
      if (consoleErr == aErrNone) {
        break;
      } else if (consoleErr == aErrTimeout) {
        aConsole_DisplayLine(pConsole, "retrying XMODEM packet", 
        		     kDisplayStatus);
        consoleErr = aErrNone;
      } else {
#ifndef aPALM
        sprintf(num, "unknown error %d", consoleErr);
        aConsole_DisplayLine(pConsole, num, 
        		     kDisplayStatus);
#endif
      }
    } 
    
    /* see if we exceeded the retry threshold */
    if ((consoleErr == aErrNone) && (i >= 10)) {
      aConsole_DisplayLine(pConsole, "retrying threshold exceeded", 
        		   kDisplayStatus);
      consoleErr = aErrCancel;
    }

    /* log the packet */
    if (consoleErr == aErrNone) {
#ifndef aPALM
      sprintf(num, "packet %d done", packet++);
      aStream_WriteLine(pConsole->ioLib, debug_log, num, NULL);
#endif
    }

#ifndef aPALM
    /* display some status */
    if (consoleErr == aErrNone) {
      if ((((float)xmodem.nWritten
            /(float)xmodem.nSize)) > (percent + 0.01f)) {
        char line[100];
        percent += 0.01f;
        sprintf(line, "%.2f\%", percent * 100);
        aConsole_DisplayLine(pConsole, line, kDisplayInput);
      }
    }
#endif
  } /* while */

  /* close the file no matter what happened */
  if (file)
    aFile_Close(pConsole->ioLib, file, NULL); 

  if (debug_log) {
    aStream_Destroy(pConsole->ioLib, debug_log, NULL);
    debug_log = NULL;
  }

  return consoleErr;

} /* aConsoleTerminal_XMODEM_Send */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMODEM_Init
 */

aErr sXMODEM_Init(aXMODEM* pXMODEM,
		  aConsole* pConsole,
		  aStreamRef stream,
		  aFileRef fileRef)
{
  aErr xErr = aErrNone;

  aBZero(pXMODEM, sizeof(aXMODEM));

  pXMODEM->pConsole = pConsole;
  pXMODEM->ioRef = pConsole->ioLib;
  pXMODEM->data[0] = SOH;
  pXMODEM->nSequence = 1;
  pXMODEM->fileRef = fileRef;
  pXMODEM->nWritten = 0;
  pXMODEM->stream = stream;

  /* find the file's size  */
  if (xErr == aErrNone)
    aFile_GetSize(pConsole->ioLib, fileRef, 
    		  &pXMODEM->nSize, &xErr);

  return xErr;

} /* sXMODEM_Init */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMODEM_Next
 */

aErr sXMODEM_Next(aXMODEM* pXMODEM)
{
  aErr xErr = aErrNone;
  int packetBytes = 128; /* maximum packet size */
  int i;
  unsigned char checksum;

  /* see if we are done */
  if (pXMODEM->nWritten >= pXMODEM->nSize)
    xErr = aErrEOF;

  /* build the next packet */
  if (xErr == aErrNone) {
    if (packetBytes > (pXMODEM->nSize - pXMODEM->nWritten))
      packetBytes = (int)(pXMODEM->nSize - pXMODEM->nWritten);
    aAssert(packetBytes >= 0);
  }

  /* set up the sequence and compliment */
  pXMODEM->data[1] = pXMODEM->nSequence;
  pXMODEM->data[2] = (unsigned char)
  		     (0xFF - (0xFF & pXMODEM->nSequence));

  /* get the bytes from the file */
  aFile_Read(pXMODEM->ioRef, 
  	     pXMODEM->fileRef, 
  	     (char*)&pXMODEM->data[3],
    	     (unsigned long)packetBytes, NULL, &xErr);

  if (xErr == aErrNone) {
    /* pad out the packet if needed */
    if (packetBytes < 128) {
      for (i = (int)packetBytes; i < 128; i++)
        pXMODEM->data[i + 3] = (unsigned char)EOF;
    }

    /* compute the checksum */
    checksum = 0;
    for (i = 0; i < 128; i++)
      checksum += pXMODEM->data[i + 3];
    pXMODEM->data[131] = checksum;

    /* record where we are in the file */  
    pXMODEM->nWritten += (unsigned long)packetBytes;
    
    /* advance the sequence pointer */
    pXMODEM->nSequence++;
  }

  return xErr;

} /* sXMODEM_Next */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMODEM_Write
 */

aErr sXMODEM_Write(aXMODEM* pXMODEM)
{
  aErr xErr = aErrNone;
  int i, cur;
  char num[20];
  char line[200];

  /* ship the packet with a little delay */
  for (cur = 0, i = 0; (xErr == aErrNone) && (i < 132); i++) {

    aIO_MSSleep(pXMODEM->ioRef, 1, NULL);

    aStream_Write(pXMODEM->ioRef,
    		  pXMODEM->stream,
    		  (char*)&pXMODEM->data[i], 1,
    		  &xErr);

    if (xErr != aErrNone)
      aStream_WriteLine(pXMODEM->ioRef, debug_log, "write error", NULL);

    switch (i) {

    case 2:
#ifndef aPALM
      sprintf(num, ">> %.2X, %.2X, %.2X", 
      	      pXMODEM->data[0], pXMODEM->data[1], pXMODEM->data[2]);
      aStream_WriteLine(pXMODEM->ioRef, debug_log, num, NULL);
#endif
      break;

    case 131:
#ifndef aPALM
      sprintf(num, "     checksum %.2X", pXMODEM->data[131]);
      aStream_WriteLine(pXMODEM->ioRef, debug_log, num, NULL);
#endif
      break;

    default:
      if (cur++ == 0)
        aStringCopy(line, "     ");
#ifndef aPALM
      sprintf(num, "%.2X ", pXMODEM->data[i]);
#endif
      if (cur == 16) {
        aStream_WriteLine(pXMODEM->ioRef, debug_log, line, NULL);
        cur = 0;
      } else {
        aStringCat(line, num);
      }
      break;
    }
  }

  /* now, look for the response ACK */
  aIO_MSSleep(pXMODEM->ioRef, 5, NULL);

  if (xErr == aErrNone)
    xErr = sXMODEM_GetResponse(pXMODEM->pConsole, ACK, 1000);

  if (xErr != aErrNone)
    aStream_WriteLine(pXMODEM->ioRef, debug_log, "read error", NULL);

  return xErr;

} /* sXMODEM_Write */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMODEM_Close
 */

aErr sXMODEM_Close(aXMODEM* pXMODEM)
{
  aErr xErr = aErrNone;

  /* send the EOT */
  aAssert(pXMODEM->nSize == pXMODEM->nWritten);

  if (xErr == aErrNone) {
    char eot[1];
    eot[0] = EOT;
    aStream_Write(pXMODEM->ioRef, pXMODEM->stream, eot, 1, &xErr);
  }

  /* now, look for the response ACK */
  if (xErr == aErrNone)
    xErr = sXMODEM_GetResponse(pXMODEM->pConsole, ACK, 20000);
  
  return xErr;

} /* sXMODEM_Close */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sXMODEM_GetResponse
 *
 * Waits up to the specified amount of time for a NAK or CAN byte.
 * Ignores all other bytes.
 */

aErr sXMODEM_GetResponse(aConsole* pConsole, 
		         const unsigned char desired,
		         const unsigned long nMSTimeout)
{
  aErr xErr = aErrNone;
  aErr replyErr;
  unsigned long now, done;
  aBool bDone = aFalse;
  aIOLib ioRef = pConsole->ioLib;
  char c;
  char line[200];

  aIO_GetMSTicks(ioRef, &done, &xErr);
  done += nMSTimeout;
  do {
    aAssert(pConsole->linkStreamRef);
    aStream_Read(ioRef, pConsole->linkStreamRef, 
    		 &c, 1, &replyErr);
    switch (replyErr) {

    case aErrNone:
#ifndef aPALM
      sprintf(line, "<< %.2X", c);
#endif
      aStream_WriteLine(pConsole->ioLib, debug_log, line, NULL);

      if (c == desired) {
        bDone = aTrue;
        break;
      } else if (c == CAN) {
        aStream_WriteLine(pConsole->ioLib, debug_log, "<< CANCEL", NULL);
        xErr = aErrCancel;
        bDone = aTrue;
        break;
      }
      break;

    case aErrNotReady:
      /* stall for a bit to avoid swamping the CPU */
      pConsole->nTerminalFlags = 1;
      aConsole_TimeSlice(pConsole, &xErr);
      pConsole->nTerminalFlags = 0;
      break;

    default:
      xErr = replyErr;
      break;
    } /* switch */

    if (xErr == aErrNone) {
      aIO_GetMSTicks(ioRef, &now, &xErr);
      if ((xErr == aErrNone) && (now > done))
        xErr = aErrTimeout;
    }

#if defined(aWIN) || defined(aWINCE)
    /* get events every 10ms */
    if ((xErr == aErrNone) && !(now % 10))
{
  MSG lpMsg;

  /* check for a message */
  while (PeekMessage(&lpMsg, NULL, 0, 0, 
    		    PM_NOYIELD | PM_NOREMOVE)) {
    if (GetMessage(&lpMsg, (HWND)NULL, 0, 0) > 0) {
      if (!IsDialogMessage((HWND)NULL, &lpMsg)) {
        TranslateMessage(&lpMsg);
        DispatchMessage(&lpMsg);
      } /* if non-dialog message */
    }
  }
}
#endif
  } while ((bDone == aFalse) && (xErr == aErrNone));

  return xErr;

} /* sXMODEM_NAK_Wait */

#endif /* aTERMINAL */
