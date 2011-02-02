/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSP03.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of cross-platform SP03 Speech 	   */
/*		module interface routines.			   */
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
#include "aSP03.h"
#include "aCmd.tea"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * parsing error definitions
 */

#define aSP03ERR_MISSING_NUM		(tkType)128
#define aSP03ERR_MISSING_STRING		(tkType)129
#define aSP03ERR_EXPECTING_NUM		(tkType)130
#define aSP03ERR_NUM_RANGE		(tkType)131
#define aSP03ERR_EXPECTING_STRING	(tkType)132
#define aSP03ERR_STRING_LENGTH		(tkType)133


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local type definitions
 */

typedef struct aSP03PreDefinedPhrase {
  unsigned char 	nVolume;
  unsigned char 	nPitch;
  unsigned char 	nSpeed;
  char			text[aSP03_MAXSTRINGLEN + 1];
} aSP03PreDefinedPhrase;

typedef struct aSP03PhraseBlock {
  unsigned char         nPhrases;
  aSP03PreDefinedPhrase rPhrases[aSP03_MAXNUMPHRASES];
  aIOLib		ioRef;
  aStreamRef		sourceStream;
  const char*		sourceName;
  aStreamRef		sp03Stream;
  aTokenizerRef		tr;
  unsigned int		nErrors;
  unsigned int		nDataSize;
  aBool			bPackFull;
  union	{
    char		b[2];
    short		w;
  } x;
  aStreamRef		output;
} aSP03PhraseBlock;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sSP03_Receive(aIOLib ioRef,
		   aStreamRef stream,
		   const unsigned char* pByte,
		   const unsigned long maxWait);
static aErr sSP03_Transmit(aIOLib ioRef,
		    aStreamRef stream,
		    const unsigned char byte,
		    const unsigned char reply);
static void sSP03TokenErr(aSP03PhraseBlock* pPhrases,
	                  tkType error,
	                  aToken* pToken);
static aErr sSP03GetRangedUChar(aSP03PhraseBlock* pPhrases,
		                const unsigned char max,
				unsigned char* pVal,
				const aBool bRequired);
static aErr sSP03ParseErrProc(tkError error,
			      const unsigned int nLine,
			      const unsigned int nColumn,
			      const unsigned int nData,
			      const char* data[],
			      void* errProcRef);
static aErr sSP03ParseChat(aSP03PhraseBlock* pPhrases);
static aErr sSP03WriteCompressedByte(aSP03PhraseBlock* pPhrases,
				     char byte);
static aErr sSP03LoadPhrases(aSP03PhraseBlock* pPhrases);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03_Receive
 *
 * This static local routine awaits a byte on the specified stream
 * for at most the specified time.  If it doesn't get there in time,
 * the routine returns a timeout.
 */

aErr sSP03_Receive(aIOLib ioRef,
		   aStreamRef stream,
		   const unsigned char* pByte,
		   const unsigned long maxWait)
{
  aErr sp03Err = aErrNone;
  aErr replyErr;
  unsigned long now, done;
  aBool bDone = aFalse;

  aIO_GetMSTicks(ioRef, &done, &sp03Err);
  done += maxWait;
  do {  
    aStream_Read(ioRef, stream, (char*)pByte, 1, &replyErr);
    switch (replyErr) {
    case aErrNone:
      bDone = aTrue;
      break;
    case aErrNotReady:
      break;
    default:
      sp03Err = replyErr;
      break;
    } /* switch */

    if (sp03Err == aErrNone) {
      aIO_GetMSTicks(ioRef, &now, &sp03Err);
      if ((sp03Err == aErrNone)
          && (now > done))
        sp03Err = aErrTimeout;
    }

  } while ((bDone == aFalse) && (sp03Err == aErrNone));
 
  return sp03Err;

} /* sSP03_Receive */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03_Transmit
 *
 * This static local routine sends the specified byte and then 
 * awaits the response byte.  Errors occur on timeout or when
 * the reply is not correct.
 */

aErr sSP03_Transmit(aIOLib ioRef,
		    aStreamRef stream,
		    const unsigned char byte,
		    const unsigned char reply)
{
  aErr sp03Err = aErrNone;
  unsigned char deviceReply;
  unsigned long	ulTimeout = aSP03_CHARTIMEOUT;
  
  /* fixed phrases need a long timeout */
  if ((byte >= 1) && (byte <= aSP03_MAXNUMPHRASES)) {
    ulTimeout = aSP03_PHRASETIMEOUT;
  }
  
  if (sp03Err == aErrNone)
    aStream_Write(ioRef, stream, (char*)&byte, 1, &sp03Err);
  
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Receive(ioRef, stream, 
    			    (unsigned char*)&deviceReply, 
    			    ulTimeout);
  
  if ((sp03Err == aErrNone) && (deviceReply != reply))
    sp03Err = aErrIO;

  return sp03Err;

} /* sSP03_Transmit */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03TokenErr
 */

void sSP03TokenErr(aSP03PhraseBlock* pPhrases,
	           tkType error,
	           aToken* pToken)
{
  /* only report if there is an output stream */
  if (pPhrases->output) {
    const char* msg[1];
    unsigned int nLine = 0;
    unsigned int nColumn = 0;
    
    msg[0] = (char*)pPhrases->sourceName;

    /* if there is a token, go get it's stream position */
    if (pToken) {
      aTokenInfo info;
      if (!aToken_GetInfo(pPhrases->ioRef, pToken, &info, NULL)) {
        nLine = info.nLine;
	nColumn = info.nColumn;
	msg[0] = info.pSourceName;
      }
    }
    sSP03ParseErrProc(error, nLine, nColumn, 1, msg, pPhrases);
  }

} /* sSP03TokenErr */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03GetRangedUChar
 */

aErr sSP03GetRangedUChar(aSP03PhraseBlock* pPhrases,
		         const unsigned char max,
		         unsigned char* pVal,
			 const aBool bRequired)
{
  aErr sp03Err = aErrNone;
  aToken* pToken = NULL;
  int raw;
  
  if (sp03Err == aErrNone)  {
    if (aTokenizer_Next(pPhrases->ioRef, 
			pPhrases->tr, &pToken, NULL)) {
      if (bRequired == aTrue) {
        sSP03TokenErr(pPhrases, aSP03ERR_MISSING_NUM, NULL);
	sp03Err = aErrParse;
      } else {
        sp03Err = aErrEOF;
      }
    }
  }

  /* ensure it was a number and if so, set raw */
  if (sp03Err == aErrNone) {
    if (pToken->eType == tkInt) {
      raw = pToken->v.integer;
    } else {
      sSP03TokenErr(pPhrases, aSP03ERR_EXPECTING_NUM, pToken);
      sp03Err = aErrParse;
    }
  }
  
  /* range check an assign return value */
  /* ensure it was a number and if so, set raw */
  if (sp03Err == aErrNone) {
    if ((raw < 0) || (raw > max)) {
      sSP03TokenErr(pPhrases, aSP03ERR_NUM_RANGE, pToken);
      sp03Err = aErrParse;
    } else {
      *pVal = (unsigned char)raw;
    }
  }

  /* clean up the token in any case */
  if (pToken != NULL)
    aTokenizer_Dispose(pPhrases->ioRef, pPhrases->tr, pToken, NULL);

  return sp03Err;

} /* sSP03GetRangedUChar */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03ParseErrProc
 */

aErr sSP03ParseErrProc(tkError error,
		       const unsigned int nLine,
		       const unsigned int nColumn,
		       const unsigned int nData,
		       const char* data[],
		       void* errProcRef)
{
  aErr sp03Err = aErrNone;
  aSP03PhraseBlock* pPhrases = (aSP03PhraseBlock*)errProcRef;
  char msg[100];
  char num[10];

  aAssert(pPhrases);
  aAssert(pPhrases->output);

  /* header */
  aStream_WriteLine(pPhrases->ioRef,
  		    pPhrases->output,
  		    "SP03 loader error:",
  		    &sp03Err);

  /* show the error location */
  
  /* the filename is always the first data parameter */
  if ((sp03Err == aErrNone) && (nData > 0)) {
    aStringCopySafe(msg, 100, " file: ");
    aStringCatSafe(msg, 100, data[0]);
    aStream_WriteLine(pPhrases->ioRef,
  		      pPhrases->output, msg, &sp03Err);
  }

  if (sp03Err == aErrNone) {        
    aStringCopySafe(msg, 100, " line: ");
    aStringFromInt(num, nLine);
    aStringCatSafe(msg, 100, num);
    aStream_WriteLine(pPhrases->ioRef,
  		      pPhrases->output, msg, &sp03Err);
  }
    
  if (sp03Err == aErrNone) {
    aStringCopySafe(msg, 100, " character: ");
    aStringFromInt(num, nColumn);
    aStringCatSafe(msg, 100, num);
    aStream_WriteLine(pPhrases->ioRef,
  		      pPhrases->output, msg, &sp03Err);
  }

  /* show the actual error */  
  if (sp03Err == aErrNone) {
    aStringCopySafe(msg, 100, " ");

    switch (error) {

    case aSP03ERR_MISSING_NUM:
      aStringCatSafe(msg, 100, "missing value");
      break;

    case aSP03ERR_MISSING_STRING:
      aStringCatSafe(msg, 100, "missing string");
      break;

    case aSP03ERR_EXPECTING_NUM:
      aStringCatSafe(msg, 100, "number expected");
      break;

    case aSP03ERR_EXPECTING_STRING:
      aStringCatSafe(msg, 100, "string expected");
      break;

    case aSP03ERR_NUM_RANGE:
      aStringCatSafe(msg, 100, "value out of range");
      break;

    case aSP03ERR_STRING_LENGTH:
      aStringCatSafe(msg, 100, "string too long");
      break;

    case tkErrUntermCmnt:
    case tkErrIncludeNotFnd:
    case tkErrDuplicateDefine:
    case tkErrBadArgumentList:
      break;

    default:
      if (nData > 1)
        aStringCatSafe(msg, 100, data[1]);
      break;

    } /* switch */    

    aStream_WriteLine(pPhrases->ioRef,
  		      pPhrases->output,
  		      msg,
  		      &sp03Err);
  }

  /* increment error count */
  pPhrases->nErrors++;

  return sp03Err;

} /* sSP03ParseErrProc */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03ParseChat
 */

aErr sSP03ParseChat(aSP03PhraseBlock* pPhrases)
{
  aErr sp03Err = aErrNone;
  unsigned int nPhrases = 0;

  /* build a tokenizer for the input stream */
  if (sp03Err == aErrNone)
    aTokenizer_Create(pPhrases->ioRef, 
		      pPhrases->sourceStream, 
		      pPhrases->sourceName,
		      aFileAreaSystem,
		      sSP03ParseErrProc, pPhrases,
		      &pPhrases->tr, &sp03Err);

  /* step through and gather up all the phrases */
  while ((sp03Err == aErrNone) 
         && (nPhrases < aSP03_MAXNUMPHRASES)) {
    aSP03PreDefinedPhrase* pPhrase = &pPhrases->rPhrases[nPhrases];

    /* get the volume or we are done */
    sp03Err = sSP03GetRangedUChar(pPhrases, aSP03_MAXVOLUME,
				  &pPhrase->nVolume, aFalse);

    /* get the pitch (required) */
    if (sp03Err == aErrNone) {
      sp03Err = sSP03GetRangedUChar(pPhrases, aSP03_MAXPITCH,
				    &pPhrase->nPitch, aTrue);
    }

    /* get the speed (required) */
    if (sp03Err == aErrNone) {
      sp03Err = sSP03GetRangedUChar(pPhrases, aSP03_MAXSPEED,
				    &pPhrase->nSpeed, aTrue);
    }
    
    /* get the phrase string */
    if (sp03Err == aErrNone) {
      aToken* pToken = NULL;
      if (aTokenizer_Next(pPhrases->ioRef, 
			  pPhrases->tr, &pToken, NULL)) {
	sSP03TokenErr(pPhrases, aSP03ERR_MISSING_STRING, NULL);
	sp03Err = aErrParse;
      } else if (pToken->eType != tkString) {
	sSP03TokenErr(pPhrases, aSP03ERR_EXPECTING_STRING, NULL);
	sp03Err = aErrParse;
      } else if (aStringLen(pToken->v.string) > aSP03_MAXSTRINGLEN) {
	sSP03TokenErr(pPhrases, aSP03ERR_STRING_LENGTH, NULL);
	sp03Err = aErrParse;
      } else {
        aStringCopySafe(pPhrase->text, aSP03_MAXSTRINGLEN + 1, pToken->v.string);
      }
      if (pToken != NULL)
        aTokenizer_Dispose(pPhrases->ioRef, pPhrases->tr, pToken, 
			   NULL);
    }

    /* account for the data size */
    if (sp03Err == aErrNone) {
      pPhrases->nDataSize += 
      		(unsigned int)(4 + aStringLen(pPhrase->text));
      nPhrases++;
    }
  }

  /* EOF is ok */
  if (sp03Err == aErrEOF)
    sp03Err = aErrNone;
    
  /* record how many valid phrases there are */
  if (sp03Err == aErrNone)
    pPhrases->nPhrases = (unsigned char)nPhrases;

  /* summarize the parsing */
  if (pPhrases->output) {
    char line[100];
    if (sp03Err == aErrNone) {
      aSNPRINTF(line, 100, "parsed %d phrases", 
                          (int)nPhrases);
      aStream_WriteLine(pPhrases->ioRef, pPhrases->output, line, 
		        NULL);
      aSNPRINTF(line, 100, "compressed into %d bytes", 
                          (int)pPhrases->nDataSize);
    } else {
      aSNPRINTF(line, 100, "there were a total of %d errors", 
                          (int)pPhrases->nErrors);
    }
    aStream_WriteLine(pPhrases->ioRef, pPhrases->output, line, 
		      NULL);
  }

  /* clean up the tokenizer */
  if (pPhrases->tr) {
    aTokenizer_Destroy(pPhrases->ioRef, pPhrases->tr, NULL);
    pPhrases->tr = NULL;
  }
  
  return sp03Err;

} /* sSP03ParseChat */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03WriteCompressedByte
 */

aErr sSP03WriteCompressedByte(aSP03PhraseBlock* pPhrases,
			      char byte)
{
  aErr sp03Err = aErrNone;

  if (pPhrases->bPackFull) {
    char reply;
    
    /* pack up the second byte */
    pPhrases->x.b[0] = (char)(byte << 1);
    pPhrases->x.w >>= 1;
    pPhrases->bPackFull = aFalse;

    /* send out the next packed word */
    if (!aStream_Write(pPhrases->ioRef, pPhrases->sp03Stream, 
		       &pPhrases->x.b[1], 1, &sp03Err))
      aStream_Write(pPhrases->ioRef, pPhrases->sp03Stream, 
	            &pPhrases->x.b[0], 1, &sp03Err);

    /* wait for the ACK byte */
    if (sp03Err == aErrNone)
      sp03Err = sSP03_Receive(pPhrases->ioRef, pPhrases->sp03Stream, 
    			      (unsigned char*)&reply, 
    			      aSP03_CHARTIMEOUT);
  } else {
    pPhrases->x.b[1] = byte;
    pPhrases->bPackFull = aTrue;
  }

  return sp03Err;

} /* sSP03WriteCompressedByte */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSP03LoadPhrases
 */

aErr sSP03LoadPhrases(aSP03PhraseBlock* pPhrases)
{
  aErr sp03Err = aErrNone;
  int i;
  
  /* initiate the programming sequence */
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Transmit(pPhrases->ioRef, pPhrases->sp03Stream,
			     aSP03_SERIALCMDLOAD, 0x02);

  /* write the lines */
  for (i = 0; (sp03Err == aErrNone) 
	      && (i < aSP03_MAXNUMPHRASES); i++) {
    aSP03PreDefinedPhrase* pPhrase = &pPhrases->rPhrases[i];
    int len = (int)aStringLen(pPhrase->text);
    int c;
    
    /* write the volume, pitch, and speed */
    if (sp03Err == aErrNone)
      sp03Err = sSP03WriteCompressedByte(pPhrases, 						         
      					 (char)pPhrase->nVolume);
    if (sp03Err == aErrNone)
      sp03Err = sSP03WriteCompressedByte(pPhrases, 						         
      					 (char)pPhrase->nPitch);
    if (sp03Err == aErrNone)
      sp03Err = sSP03WriteCompressedByte(pPhrases,		
					 (char)pPhrase->nSpeed);
 
    /* followed by the text and terminator */
    for (c = 0; (sp03Err == aErrNone) && (c <= len); c++)
      sp03Err = sSP03WriteCompressedByte(pPhrases,		
					 (char)pPhrase->text[c]);
  } /* for */

  /* finish the programming sequence */
  if (sp03Err == aErrNone)
      sp03Err = sSP03WriteCompressedByte(pPhrases, (char)0xFF);

  /* check to see if there is an odd byte */
  if ((pPhrases->bPackFull) && (sp03Err == aErrNone))
    sp03Err = sSP03WriteCompressedByte(pPhrases, (char)0xFF);

  /* message where appropriate */
  if ((pPhrases->output) && (sp03Err == aErrNone)) {
    char line[100];
    aSNPRINTF(line, 100, "loaded %d phrases", i);
    aStream_WriteLine(pPhrases->ioRef, pPhrases->output, line, 
		      NULL);
  }

  return sp03Err;

} /* sSP03LoadPhrases */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSP03Direct_GetVersion
 */

aErr aSP03Direct_GetVersion(aStreamRef sp03stream,
		            unsigned char* pSoundChipHardware,
		            unsigned char* pSoundChipSoftware,
		            unsigned char* pModuleFirmware)
{
  aErr sp03Err = aErrNone;
  aIOLib ioRef = 0;
  char cmd;
  
  /* check all the parameters and get set up */
  if (!sp03stream || !pSoundChipHardware 
      || !pSoundChipSoftware || !pModuleFirmware)
    sp03Err = aErrParam; 
  if (sp03Err == aErrNone) {
    ioRef = aStreamLibRef(sp03stream);
    if (!ioRef)
      sp03Err = aErrConfiguration;
  }

  /* send the version request byte */
  cmd = (char)aSP03_SERIALCMDVERSION;
  if (sp03Err == aErrNone)
    aStream_Write(ioRef, sp03stream, &cmd, 1, &sp03Err);

  if (sp03Err == aErrNone)
    sp03Err = sSP03_Receive(ioRef, sp03stream, 
    			    (unsigned char*)pSoundChipHardware, 
    			    aSP03_CHARTIMEOUT);
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Receive(ioRef, sp03stream, 
    			    (unsigned char*)pSoundChipSoftware, 
    			    aSP03_CHARTIMEOUT);
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Receive(ioRef, sp03stream, 
    			    (unsigned char*)pModuleFirmware, 
    			    aSP03_CHARTIMEOUT);

  return sp03Err;

} /* aSP03Direct_GetVersion */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSP03Direct_SpeakPhrase
 */

aErr aSP03Direct_SpeakPhrase(aStreamRef sp03stream,
		             const unsigned char nPhrase)
{
  aErr sp03Err = aErrNone;
  aIOLib ioRef;
  
  /* check all the parameters and get set up */
  if (!sp03stream)
    sp03Err = aErrParam; 
  if ((sp03Err == aErrNone)
      && (nPhrase > aSP03_MAXNUMPHRASES))
    sp03Err = aErrRange;
  if (sp03Err == aErrNone) {
    ioRef = aStreamLibRef(sp03stream);
    if (!ioRef)
      sp03Err = aErrConfiguration;
  }

  /* now do the speech */
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Transmit(ioRef, sp03stream, nPhrase, nPhrase);

  return sp03Err;

} /* aSP03Direct_SpeakPhrase */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSP03Direct_SpeakString
 */

aErr aSP03Direct_SpeakString(aStreamRef sp03stream,
		             const unsigned char nVolume,
		             const unsigned char nPitch,
		             const unsigned char nSpeed,
		             const char* pString)
{
  aErr sp03Err = aErrNone;
  aIOLib ioRef = 0;
  unsigned long len = 0;
  unsigned long i;
  
  /* check all the parameters and get set up */
  if (!sp03stream || !pString)
    sp03Err = aErrParam; 
  if (sp03Err == aErrNone) {
    len = aStringLen(pString);
    if (len > aSP03_MAXSTRINGLEN)
      sp03Err = aErrParam;
  }
  if (sp03Err == aErrNone) {
    ioRef = aStreamLibRef(sp03stream);
    if (!ioRef)
      sp03Err = aErrConfiguration;
  }

  /* now do the speech */
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Transmit(ioRef, sp03stream, 0x80, 0x01);
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Transmit(ioRef, sp03stream, nVolume, nVolume);
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Transmit(ioRef, sp03stream, nPitch, nPitch);
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Transmit(ioRef, sp03stream, nSpeed, nSpeed);
  for (i = 0; (i < len) && (sp03Err == aErrNone); i++)
    sp03Err = sSP03_Transmit(ioRef, sp03stream, 
    			     (unsigned char)pString[i], 
    			     (unsigned char)pString[i]);
  if (sp03Err == aErrNone)
    sp03Err = sSP03_Transmit(ioRef, sp03stream, 0x00, 0x00);

  /* finally, wait for the reply to show the end */
  if (sp03Err == aErrNone) {
    unsigned char reply;
    sp03Err = sSP03_Receive(ioRef, sp03stream, 
    			    (unsigned char*)&reply, 
    			    len * aSP03_CHARTIMEOUT);
    if ((sp03Err == aErrNone) && (reply != 0x00))
      sp03Err = aErrIO;
  }

  return sp03Err;

} /* aSP03Direct_SpeakString */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSP03Direct_LoadPredefines
 */

aErr aSP03Direct_LoadPredefines(aStreamRef phraseStream,
				const char* phraseStreamName,
		                aStreamRef sp03Stream,
				aStreamRef outputStream)
{
  aErr sp03Err = aErrNone;
  aIOLib ioRef = 0;
  aSP03PhraseBlock* pPhrases = NULL;
  
  /* check the parameters */
  if (!sp03Stream || !phraseStream || !phraseStreamName)
    sp03Err = aErrParam;
  else {
    ioRef = aStreamLibRef(sp03Stream);
    if (!ioRef) {
      sp03Err = aErrParam;
    }
  }
  
  /* build the phrase block for parsing into and loading from */
  if (sp03Err == aErrNone) {
    pPhrases = (aSP03PhraseBlock*)
		      aMemAlloc(sizeof(aSP03PhraseBlock));
    if (!pPhrases)
      sp03Err = aErrNone;
    else {
      aBZero(pPhrases, sizeof(aSP03PhraseBlock));
      pPhrases->ioRef = ioRef;
      pPhrases->sourceStream = phraseStream;
      pPhrases->sourceName = phraseStreamName;
      pPhrases->sp03Stream = sp03Stream;
      pPhrases->output = outputStream;
    }
  }

  /* post a compiler message */
  if ((sp03Err == aErrNone) && (pPhrases->output)) {
    if (!aStream_WriteLine(ioRef, pPhrases->output, 
	                   "Devantech SP03 phrase loader",
			   &sp03Err))
      aStream_WriteLine(ioRef, pPhrases->output, 
		        "Copyright 2002-2004, Acroname Inc.",
			&sp03Err);
  }

  /* parse the input stream for the phrases */
  if (sp03Err == aErrNone)
    sp03Err = sSP03ParseChat(pPhrases);

  /* write the phrases to the SP03 module */
  if (sp03Err == aErrNone)
    sp03Err = sSP03LoadPhrases(pPhrases);

  /* clean up no matter what happened */
  if (pPhrases)
    aMemFree(pPhrases);

  return sp03Err;

} /* aSP03Direct_LoadPredefines */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSP03_SpeakPhrase
 */

aErr aSP03_SpeakPhrase(aStemLib stemLib,
		       const unsigned char nPhrase)
{
  aErr sp03Err = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;

  /* check all the parameters and get set up */
  if (!stemLib)
    sp03Err = aErrParam; 
  if ((sp03Err == aErrNone)
      && (nPhrase > aSP03_MAXNUMPHRASES))
    sp03Err = aErrRange;

  /* send the pre-loaded phrase we need */
  if (sp03Err == aErrNone) {
    data[0] = aSP03_IICCOMMAND;
    data[1] = (char)nPhrase;
    if (!aPacket_Create(stemLib, 
    			aSP03_IICADDRESS, 
    			2, 
    			data,
    			&packet, 
    			&sp03Err))
      aStem_SendPacket(stemLib, packet, &sp03Err);
  }
  
  return sp03Err;

} /* aSP03_SpeakPhrase */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSP03_SpeakString
 */

aErr aSP03_SpeakString(aStemLib stemLib,
		       const unsigned char nVolume,
		       const unsigned char nPitch,
		       const unsigned char nSpeed,
		       const char* pString)
{
  aErr sp03Err = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
  unsigned int len;
  unsigned char cur;
  const char* p = pString;
  int left = 0;


  /* check all the parameters and get set up */
  if (!stemLib || !pString)
    sp03Err = aErrParam; 
  if (sp03Err == aErrNone) {
    len = (unsigned int)aStringLen(pString);
    if (len > aSP03_MAXSTRINGLEN)
      sp03Err = aErrRange;
    else
      left = (int)len + 1; /* include the null terminator */
  }

  /* the first packet has the speech parameters in front */
  if (sp03Err == aErrNone) {
    data[0] = aSP03_IICCOMMAND;
    data[1] = aSP03_CMDNOP;
    data[2] = (char)nVolume;
    data[3] = (char)nPitch;
    data[4] = (char)nSpeed;
    cur = 5;
    
    /* send the text broken up into packets */
    do {

      /* fill up the rest of the packet */
      while ((cur < aSTEMMAXPACKETBYTES)
             && (left-- > 0)) {
        data[cur] = *p++;
        cur++;
      }

      /* send the packet */
      if (!aPacket_Create(stemLib, 
    			  aSP03_IICADDRESS, 
    			  cur, 
    			  data,
    			  &packet, 
    			  &sp03Err))
        aStem_SendPacket(stemLib, packet, &sp03Err);

      /* reset for the next packet */
      if (left > 0) {
        cur = 2;
      }
      
    } while ((sp03Err == aErrNone) && (left > 0));
  }

  /* now, send the speak buffer command */
  if (sp03Err == aErrNone) {
    data[0] = aSP03_IICCOMMAND;
    data[1] = aSP03_CMDSPKBUF;
    if (!aPacket_Create(stemLib, 
    			aSP03_IICADDRESS, 
    			2, 
    			data,
    			&packet, 
    			&sp03Err))
      aStem_SendPacket(stemLib, packet, &sp03Err);
  }

  return sp03Err;

} /* aSP03_SpeakString */
