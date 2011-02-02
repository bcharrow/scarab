/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemTEAFile.c                                 	   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: cross-platform TEA file stream                     */
/*              implementation.                                    */
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

#include "aStemInternal.h"

#define aTEAFILETIMEOUTMS		2000


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * structure used by the stream interface in the create input/output
 * routines for TEA program files.
 */

typedef struct aTEAFileState {

  /* we communicate through the stem stream */
  aStemLib		stemRef;
  
  /* holds the bytes being assembled for sending or bytes received
   * in the last packet that have not been used yet */
  char			packetBuffer[aSTEMMAXPACKETBYTES];

  /* tells how many bytes are valid in the buffer */
  int			nBytesInBuffer;
  
  /* checksum used to ensure the file is transfered safely */
  unsigned char		checksum;
  
  /* used to track page boundary stuff */
  int			nBytesSent;
  
  /* set to true when the cmdTEAEOF command is received */
  aBool			bDone;

  /* address of the module being accessed in subsequent
   * cmdTEADATA or cmdTEAREAD calls */
  unsigned char		address;
  
  /* file number being written */
  int 			nFileNumber;

} aTEAFileState;




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * static local routines (callbacks)
 */

static aErr sStemTEAFile_Get(char* pData,
                             void* ref);
static aErr sStemTEAFile_Put(char* pData,
                             void* ref);
static aErr sStemTEAFile_DeleteOutput(void* ref);
static aErr sStemTEAFile_DeleteInput(void* ref);

static aBool sInitFilter(const unsigned char address,
			 const unsigned char length,
			 const char* data);
static aBool sCheckSumFilter(const unsigned char address,
			     const unsigned char length,
			     const char* data);
static aBool sDataFilter(const unsigned char address,
			 const unsigned char length,
			 const char* data);

static aErr sStemTEAFile_Init(aTEAFileState* pFileState);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStemTEAFile_Put
 */

aErr sStemTEAFile_Put(char* pData,
	   	      void* ref)
{
  aErr err = aErrNone;
  aTEAFileState* pPrivate = (aTEAFileState*)ref;
  int nPreCt;
  int* pBuffCt;
  aPacketRef outPacket;
  int nLength;


  if (pPrivate == NULL)
    err = aErrIO;
  
  if (err == aErrNone)
    pBuffCt = &(pPrivate->nBytesInBuffer);

  if (err == aErrNone) {

    /* update checksum with outgoing character */
    pPrivate->checksum += (unsigned char)(*pData);

    /* stick byte in buffer, +1 offset for command */
    pPrivate->packetBuffer[(*pBuffCt) + 1] = (*pData);
    (*pBuffCt)++;

    /* +2 offset for file size in EEPROM */
    nPreCt = (*pBuffCt) + (pPrivate->nBytesSent) + 2;

    /* if buffer full or EEPROM page boundary reached then 
     * ship packet */
    if (((*pBuffCt) == 7) || ((nPreCt % 64) == 0)) {

      /* send cmdTEADATA packet */
      nLength = (*pBuffCt) + 1;
      pPrivate->packetBuffer[0] = cmdTEADATA;
      if (!aPacket_Create(pPrivate->stemRef, 
			  pPrivate->address, 
			  (unsigned char) nLength,
			  pPrivate->packetBuffer, 
			  &outPacket, &err)) {
	aStem* pStem = (aStem*)pPrivate->stemRef;
        aErr drainErr;
        /* drain the input stream since this may be a tight loop 
         * calling us and we don't want to starve the heartbeat
         */
        drainErr = aStem_DrainStream(pPrivate->stemRef,
				     pStem->linkStream,
				     &pStem->curLinkPacket,
				     NULL, NULL,
				     aTrue,
				     aPacketFromStem,
				     pStem->pLinkPacketList);
        aAssert(drainErr == aErrNone);
      
        aStem_SendPacket(pPrivate->stemRef, outPacket, &err);
	
        /* MRW delay to get around wireless latency troubles */
        if (err == aErrNone)
          aIO_MSSleep(pStem->ioRef,
		      (unsigned long)pStem->nTEADataDelayMS,
		      NULL);
          				   
      }

      if (err == aErrNone) {
        pPrivate->nBytesSent += (*pBuffCt);
        (*pBuffCt) = 0;
      }
    }
  }

  return(err);

} /* sStemTEAFile_Put */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStemTEAFile_Get
 */

static aErr sStemTEAFile_Get(char* pData,
			      void* ref)
{
  aErr err = aErrNone;
  aTEAFileState* pPrivate = (aTEAFileState*)ref;

  int* pBuffCt;
  aPacketRef outPacket;
  aPacketRef inPacket;

  unsigned char ucLength;
  unsigned char ucAddress;
  char data[aSTEMMAXPACKETBYTES];
  char datum;

  int ii;
  int nDataSize;
  
  if (pPrivate == NULL)
    err = aErrIO;
  
  
  if ((err == aErrNone) && (pPrivate->bDone == aTrue))
    err = aErrEOF;

  pBuffCt = &(pPrivate->nBytesInBuffer);
    
  if (err == aErrNone) {  

    /* check for empty buffer */
    if (*pBuffCt == 0) {

      /* send cmdTEAREAD command */
      ucLength = 1;
      data[0] = cmdTEAREAD;
      aPacket_Create(pPrivate->stemRef, 
		     pPrivate->address, 
		     ucLength,
		     data, 
		     &outPacket, 
		     &err);
      if (err == aErrNone)
        aStem_SendPacket(pPrivate->stemRef, outPacket, &err);

      /* seek cmdTEADATA packet reply */
      if (err == aErrNone) {
        aStem_GetPacket(pPrivate->stemRef, 
			(aPacketFilter)sDataFilter,
			NULL,
			aTEAFILETIMEOUTMS,
			&inPacket, 
			&err);
        if (err == aErrNone)
          aPacket_GetData(pPrivate->stemRef,
			  inPacket,
			  &ucAddress,
			  &ucLength,
			  data,
			  &err);
        if (err == aErrNone) {
        
          if (0) {
	    aStem* pStem = (aStem*)pPrivate->stemRef;
            aPacket* pTemp = pStem->pLinkPacketList->pHead;
            char formatted[30];
            aStem_DebugLine(pPrivate->stemRef, "packetstart", NULL);
            while (pTemp != NULL) {
              aPacket_Format(pPrivate->stemRef, pTemp, formatted, 30, NULL);
              aStem_DebugLine(pPrivate->stemRef, formatted, NULL);
              pTemp = pTemp->pNext;
            }
            aStem_DebugLine(pPrivate->stemRef, "packetend", NULL);
          }

          switch (data[0]) {
  
          case cmdTEADATA:
            nDataSize = ucLength - 1;
            for (ii = 0; ii < nDataSize; ii++) {
              datum = data[nDataSize-ii];
              pPrivate->packetBuffer[ii] = datum;
              pPrivate->checksum += (unsigned char)datum;
            }
            *pBuffCt = nDataSize;
            pPrivate->nBytesSent += nDataSize;
            break;

          case cmdTEAEOF:
            pPrivate->bDone = aTrue;
            break;

          } /* switch */

          aPacket_Destroy(pPrivate->stemRef, inPacket, &err);
        }
      }
    } /* buf count = 0 */
  } /* err == aErrNone */

  /* if we are done, report the end of file condition */
  if (pPrivate->bDone == aTrue) {
    err = aErrEOF;
  } else {
  
    /* else send one character from buffer */
    if ((err == aErrNone) && (*pBuffCt > 0)) {
      (*pBuffCt)--;
      *pData = pPrivate->packetBuffer[(*pBuffCt)];
    }
  }

  return(err);

} /* sStemTEAFile_Get */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStemTEAFile_DeleteInput
 */

aErr sStemTEAFile_DeleteInput(void* ref)
{
  aErr err = aErrNone;
  aTEAFileState* pPrivate = (aTEAFileState*)ref;
  unsigned char ucLength;
  unsigned char ucAddress;
  unsigned char ucChecksum;
  char data[aSTEMMAXPACKETBYTES];

  aPacketRef inPacket;
  aPacketRef outPacket;

  if (pPrivate == NULL)
    err = aErrIO;

  /* send cmdTEASUM */
  if (err == aErrNone) {
    ucLength = 1;
    pPrivate->packetBuffer[0] = cmdTEASUM;
    aPacket_Create(pPrivate->stemRef, 
		   pPrivate->address, 
		   ucLength,
		   pPrivate->packetBuffer, 
		   &outPacket,
		   &err);
  }
  if (err == aErrNone)
    aStem_SendPacket(pPrivate->stemRef, 
		     outPacket, 
		     &err);

  /* get the checksum reply */
  if (err == aErrNone)
    aStem_GetPacket(pPrivate->stemRef, 
		    (aPacketFilter)sCheckSumFilter,
		    NULL,
		    aTEAFILETIMEOUTMS,
		    &inPacket,
		    &err);
  if (err == aErrNone)
    aPacket_GetData(pPrivate->stemRef, 
		    inPacket, 
		    &ucAddress, 
		    &ucLength, 
		    data,
		    &err);

  if (err == aErrNone)
    aPacket_Destroy(pPrivate->stemRef, inPacket, &err);
  
  if (err == aErrNone) {
    ucChecksum = (unsigned char)data[1];
    if (ucChecksum != pPrivate->checksum) 
      err = aErrRead;
  }

  /* send cmdTEAEOF */
  if (err == aErrNone) {
    ucLength = 1;
    pPrivate->packetBuffer[0] = cmdTEAEOF;
    aPacket_Create(pPrivate->stemRef, 
		   pPrivate->address, 
		   ucLength,
		   pPrivate->packetBuffer, 
		   &outPacket,
		   &err);
    if (err == aErrNone)
      aStem_SendPacket(pPrivate->stemRef, 
		       outPacket, 
		       &err);
  }

  /* clean up the private stream data */
  if (pPrivate != NULL)
    aMemFree((aMemPtr)pPrivate);

  return(err);

} /* sStemTEAFile_DeleteInput */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStemTEAFile_DeleteOutput
 */

aErr sStemTEAFile_DeleteOutput(void* ref)
{
  aErr err = aErrNone;
  aTEAFileState* pPrivate = (aTEAFileState*)ref;
  int* pBuffCt;
  unsigned char ucLength;
  unsigned char ucAddress;
  unsigned char ucChecksum;
  char data[aSTEMMAXPACKETBYTES];

  aPacketRef outPacket;
  aPacketRef inPacket;

  if (pPrivate == NULL)
    err = aErrIO;

  if (err == aErrNone) {

    /* ship any remaining bytes */
    pBuffCt = &(pPrivate->nBytesInBuffer);

    if ((*pBuffCt) > 0) {

      /* send cmdTEADATA packet */
      ucLength = (unsigned char)((*pBuffCt)+1);
      pPrivate->packetBuffer[0] = cmdTEADATA;
      aPacket_Create(pPrivate->stemRef, 
		     pPrivate->address, 
		     ucLength,
		     pPrivate->packetBuffer, 
		     &outPacket,
		     &err);
      if (err == aErrNone)
	aStem_SendPacket(pPrivate->stemRef, outPacket, &err);
      if (err == aErrNone) {
        pPrivate->nBytesSent += (*pBuffCt);
        (*pBuffCt) = 0;
      }
    }
  }

  /* send cmdTEASUM to get the checksum */
  if (err == aErrNone) {
    ucLength = 1;
    pPrivate->packetBuffer[0] = cmdTEASUM;
    aPacket_Create(pPrivate->stemRef, 
		   pPrivate->address, 
		   ucLength,
		   pPrivate->packetBuffer, 
		   &outPacket,
		   &err);
    if (err == aErrNone)
      aStem_SendPacket(pPrivate->stemRef, outPacket, &err);
  }

  /* get the checksum reply */
  if (err == aErrNone)
    aStem_GetPacket(pPrivate->stemRef, 
		    (aPacketFilter)sCheckSumFilter,
		    NULL,
		    aTEAFILETIMEOUTMS,
		    &inPacket,
		    &err);
  if (err == aErrNone)
    aPacket_GetData(pPrivate->stemRef, 
		    inPacket, 
		    &ucAddress, 
		    &ucLength, 
		    data,
		    &err);

  if (err == aErrNone)
    aPacket_Destroy(pPrivate->stemRef, inPacket, &err);
  
  if (err == aErrNone) {
    ucChecksum = (unsigned char)data[1];
    if (ucChecksum != pPrivate->checksum) 
      err = aErrWrite;
  }

  /* send cmdTEAEOF */
  /* try to send it even if we get a checksum error */
  /* it is okay to close a file after a checksum error */
  /* checksum error will be passed back to caller */
  if (err == aErrNone || err == aErrWrite) {
    ucLength = 1;
    pPrivate->packetBuffer[0] = cmdTEAEOF;
    if (!aPacket_Create(pPrivate->stemRef, 
			pPrivate->address, 
			ucLength,
			pPrivate->packetBuffer, 
			&outPacket,
			&err))
      aStem_SendPacket(pPrivate->stemRef, outPacket, &err);
  }

  /* clean up the private stream data */
  if (pPrivate != NULL)
    aMemFree((aMemPtr)pPrivate);

  return err;

} /* sStemTEAFile_DeleteOutput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sInitFilter
 */

aBool sInitFilter(const unsigned char address,
		  const unsigned char length,
		  const char* data)
{
  if (data[0] == cmdTEAINIT)
    return aTrue;

  return aFalse;

} /* sInitFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCheckSumFilter
 */

aBool sCheckSumFilter(const unsigned char address,
		              const unsigned char length,
		              const char* data)
{
  if (data[0] == cmdTEASUM)
    return aTrue;

  return aFalse;

} /* sCheckSumFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sDataFilter
 */

aBool sDataFilter(const unsigned char address,
		  const unsigned char length,
		  const char* data)
{  
  if ((data[0] == cmdTEAEOF) ||
      (data[0] == cmdTEADATA))
    return aTrue;
    
  return aFalse;

} /* sDataFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStemTEAFile_Init
 */

aErr sStemTEAFile_Init(aTEAFileState* pFileState)
{
  aErr err = aErrNone;
  aPacketRef inPacket = NULL;
  aPacketRef outPacket = NULL;
  unsigned char ucLength;
  unsigned char ucAddress;
  char data[aSTEMMAXPACKETBYTES];

  /* send cmdTEAINIT */
  if (err == aErrNone) {
    ucLength = 2;
    data[0] = cmdTEAINIT;
    data[1] = (char)pFileState->nFileNumber;
    aPacket_Create(pFileState->stemRef, 
		   pFileState->address, 
		   ucLength, 
		   data,
		   &outPacket,
		   &err);
    if (err == aErrNone)
      aStem_SendPacket(pFileState->stemRef, 
		       outPacket,
		       &err);
  }

  /* wait for init reply */
  if (err == aErrNone)
    aStem_GetPacket(pFileState->stemRef, 
		    (aPacketFilter)sInitFilter,
		    NULL,
		    aTEAFILETIMEOUTMS,
		    &inPacket,
		    &err);
  if (err == aErrNone)
    aPacket_GetData(pFileState->stemRef, 
		    inPacket, 
		    &ucAddress, 
		    &ucLength, 
		    data,
		    &err);

  if (err == aErrNone)
    aPacket_Destroy(pFileState->stemRef,
		    inPacket,
		    &err);

  /* validate the init reply */
  if (err == aErrNone) {
    /* packet data should be init, could be file error */
  }

  return err;

} /* sStemTEAFile_Init */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemInternal_CreateTEAFileInput
 * 
 * This routine creates an input stream to allow reading a TEA
 * program from a BrainStem Module.
 *
 * When this routine is called, it builds a stream interface using
 * aStream_Create. It also allocates (using aMemAlloc) a 
 * aTEAFileState structure that is defined at the top of this file.  
 * This structure is pointed to by the getRef and deleteRef 
 * of the stream interface and used by the get and delete 
 * callbacks.
 *
 * Opening the stream sends a cmdTEAINIT command packet to the 
 * addressed module to initiate the reading. The first packet is 
 * also requested using the cmdTEAREAD command.  When the first 
 * packet is obtained (using the aStemInternal_GetPacket routine), 
 * the checksum is updated and the packet bytes are stored in the 
 * state structure data buffer.
 *
 * Each get callback takes the next byte from the buffer and 
 * returns it.  When no more bytes are available in the buffer,
 * the next read packet is requested using the cmdTEAREAD command
 * within the get callback proc.
 *
 * When the last byte in the last packet is sent by the addressed
 * BrainStem module, the cmdTEAEOF packet is also sent by the 
 * module and the bDone boolean is set in the aTEAFileState 
 * structure.
 *
 * Subsequent reads will call get and it will immediately return
 * a aErrEOF error when the bDone boolean has been set.
 *
 * If the stream is destroyed before the bDone command is set, the
 * cmdTEAEOF command is sent to the module to reset the file state
 * and terminate the read operation.  The delete proc then frees 
 * up the stream state structure (type aTEAFileState) and returns
 * gracefully.
 */

aLIBRETURN
aStem_CreateTEAFileInput(aStemLib stemRef, 
			 const unsigned char address,
			 const int nFileNumber,
			 aStreamRef* pStreamRef,
			 aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aStreamRef streamRef;
  aTEAFileState* pPrivate = NULL;

  aVALIDSTEM(pStem);
  if (pStreamRef == NULL)
    err = aErrParam;
  else if (address & 0x01)
    err = aErrParam;

  /* now build the private structure */
  if (err == aErrNone) {
    pPrivate = (aTEAFileState*)aMemAlloc(sizeof(aTEAFileState));

    if (pPrivate == NULL)
      err = aErrMemory;
    else
      aBZero(pPrivate, sizeof(aTEAFileState));
  }
  
  /* fill in the structures */
  if (err == aErrNone) {      
    pPrivate->stemRef = stemRef;
    pPrivate->address = address;
    pPrivate->nFileNumber = nFileNumber;
  } 

  /* build the generic stream */
  if (err == aErrNone) {

    if (pStreamRef == NULL)
      return aErrParam;

    /* initialize the return value */
    *pStreamRef = NULL;

    aStream_Create(pStem->ioRef, 
    		   sStemTEAFile_Get,
    		   NULL,
    		   sStemTEAFile_DeleteInput,
    		   pPrivate,
    		   &streamRef, 
    		   &err);
  }

  if (err == aErrNone)
    *pStreamRef = streamRef;

  /* send and verify the init packet */
  if (err == aErrNone)
    err = sStemTEAFile_Init(pPrivate);

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStemInternal_CreateTEAFileInput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemInternal_CreateTEAFileOutput
 * 
 * This routine creates an output stream to allow writing a TEA
 * program to a BrainStem Module.
 *
 * The routine is very similar to the input analog but has the 
 * different sequence:
 *  - cmdTEAINIT initiates the write on the correct file of the 
 *               correct module
 *  - cmdTEADATA sends the packets to the stem
 *  - cmdTEAEOF signals the end of the transmission
 *
 * This routines builds a stream interface structure and a 
 * aTEAFileState structure but it only sets the put and delete
 * callback procs in the stream interface structure.
 * 
 * This routine may need to implement a delay to wait for the 
 * EEPROM write on the module. Chances are, the baud rate and 
 * packet size will allow enough time between cmdTEADATA packets
 * for the EEPROM write to occur.
 */

aLIBRETURN 
aStem_CreateTEAFileOutput(aStemLib stemRef, 
			  const unsigned char address,
			  const int nFileNumber,
			  aStreamRef* pStreamRef,
			  aErr* pErr)
{
  aErr err = aErrNone;
  aStem* pStem = (aStem*)stemRef;
  aStreamRef streamRef = NULL;
  aTEAFileState* pPrivate = NULL;


  aVALIDSTEM(pStem);
  if (pStreamRef == NULL)
    err = aErrParam;
  else if ((address & 0x01) || (address == 0x00))
    err = aErrParam;

  /* now build the private structure */  
  if (err == aErrNone) {
    pPrivate = (aTEAFileState*)aMemAlloc(sizeof(aTEAFileState));

    if (pPrivate == NULL)
      err = aErrMemory;
  }
  
  /* fill in the structures */
  if (err == aErrNone) {
    aBZero(pPrivate, sizeof(aTEAFileState));
    pPrivate->stemRef = stemRef;
    pPrivate->address = address;
    pPrivate->nFileNumber = nFileNumber;
  }

  /* send and verify the init packet */
  if (err == aErrNone)
    err = sStemTEAFile_Init(pPrivate);

  /* if there were errors, clean up the memory allocated above */
  if ((err != aErrNone) && (pPrivate != NULL))
    aMemFree((aMemPtr)pPrivate);

  /* build the generic stream */
  if (err == aErrNone) {
    aAssert(pPrivate);

    /* initialize */
    *pStreamRef = NULL;
    if (aStream_Create(pStem->ioRef, 
    		       NULL,
    		       sStemTEAFile_Put,
    		       sStemTEAFile_DeleteOutput,
    		       pPrivate,
    		       &streamRef, 
    		       &err))
      return err;
  }
 
  /* set the output variable */
  if (err == aErrNone)
    *pStreamRef = streamRef;

  if (pErr)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aStem_CreateTEAFileOutput */

