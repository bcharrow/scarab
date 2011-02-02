/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aBrainDump.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent robotics  */
/*              file format utility object.			   */
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
#include "aUtil.h"
#include "aBrainDump.h"
#include "aStream_TextLine.h"
#include "aStemCore.h"
#include "aModuleUtil.h"

#define aBDVERSION	1

#define aBD_MAXINPUTLINE		100

#define	aBD_READINC	7

#define kBDImporting	0x0001

#define			kBDObjectCup		1
#define			kBDObjectBatch		2
#define			kBDObjectPacket		3


typedef struct aBDCup {
  unsigned char		nModule;
  unsigned char		nSlot;
  unsigned short	nSize;
  char*			pCopy;
  aStreamRef		fileBuffer;
} aBDCup;



typedef struct aBDBatch {
  char			name[aFILE_NAMEMAXCHARS];
} aBDBatch;



#define 		kBDPktHasAddress	0x01
#define 		kBDPktHasLength		0x02

typedef struct aBDPacket {
  unsigned char		fStatus;
  unsigned char		nAddress;
  unsigned char		nLength;
  unsigned char		nBytes;
  char			data[aSTEMMAXPACKETBYTES];
} aBDPacket;

/* forward declarations */
typedef struct aBD* aBDPtr;
typedef struct aBDObject* aBDObjectPtr;

typedef aErr (*aBDSetNameProc)(aBDPtr pBD,
			       const char* pName);
typedef aErr (*aBDSetSlotProc)(aBDPtr pBD,
			       const unsigned char nSlot);
typedef aErr (*aBDSetModuleProc)(aBDPtr pBD,
			         const unsigned char nModule);
typedef aErr (*aBDWriteProc)(aBDPtr pBD,
	                     aBDObjectPtr pObject,
			     aStreamRef destStream);
typedef aErr (*aBDReadProc)(aBDPtr pBD,
	                    aBDObjectPtr pObject,
			    aStreamRef dumpStream);
typedef aErr (*aBDDumpProc)(aBDPtr pBD,
	                    aBDObjectPtr pObject,
			    aStemLib stemRef);
typedef aErr (*aBDCleanupProc)(aBDPtr pBD,
			       aBDObjectPtr pObject);

typedef struct aBDObject {
  short			nType;
  union {
    aBDCup		cup;
    aBDBatch		batch;
    aBDPacket		packet;
  } o;
  aBDObjectPtr		pNext;
  aBDSetNameProc	setName;
  aBDSetSlotProc	setSlot;
  aBDSetModuleProc	setModule;
  aBDWriteProc		write;
  aBDReadProc		read;
  aBDDumpProc		dump;
  aBDCleanupProc	cleanup;
} aBDObject;

typedef struct aBD {
  int			flags;
  aIOLib		ioRef;
  aStemLib		stemRef;
  aStreamRef		status;
  
  aMemPoolRef		pObjectPool;

  aBDObject*		pCurrent;
  aBDObject*		pObjects;
  aBDObject*		pTail;

  int			nErrors;

  int			check;

  aBDStatusProc		statusProc;
  void*			statusRef;

} aBD;

#define aBDCHECK 0xBDBD

#define aVALIDBD(p) if(((p) == NULL) || 			   \
		(((aBD*)p)->check != aBDCHECK)) {	           \
		  bdErr = aErrParam; }



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sBD_ParseErr(tkError error,
			 const unsigned int nLine,
			 const unsigned int nColumn,
			 const unsigned int nData,
			 const char* data[],
			 void* errProcRef);

static aErr sStart(aXMLNodeRef node,
		   const char* pKey,
		   void* vpRef);

static aErr sContent(aXMLNodeRef node,
		     const char* pKey,
		     const aToken* pToken,
		     void* vpRef);

static aErr sEnd(aXMLNodeRef node,
	         aXMLNodeRef parent,
		 const char* pKey,
		 void* vpRef);

static aErr sCupSetName(aBD* pBD,
		        const char* pName);
static aErr sCupSetSlot(aBD* pBD,
		        const unsigned char nSlot);
static aErr sCupSetModule(aBD* pBD,
		          const unsigned char nModule);
static aErr sCupCleanup(aBD* pBD,
		        aBDObject* pObject);
static aErr sCupWrite(aBD* pBD,
	              aBDObject* pCup,
	              aStreamRef destStream);
static aErr sCupRead(aBD* pBD,
	             aBDObject* pCup,
	             aStreamRef dumpStream);
static aErr sCupDump(aBD* pBD,
	             aBDObject* pCup,
	             aStemLib stemRef);

static aErr sBatchSetName(aBD* pBD,
		        const char* pName);

static aErr sPacketWrite(aBD* pBD,
			 aBDObject* pPacket,
			 aStreamRef destStream);
static aErr sPacketRead(aBD* pBD,
			aBDObject* pPacket,
			aStreamRef dumpStream);
static aErr sPacketDump(aBD* pBD,
	                aBDObject* pPacket,
			aStemLib stemRef);

static aErr sBuildObject(aBD* pBD,
			 const short nType,
			 aBDObject** ppObject);

static aErr sAddObject(aBD* pBD,
		       aBDObject* pObject);
static aErr sDeleteObject(aBD* pBD,
		          aBDObject* pObject);

static aErr sPacketInit(aBDPacket* pPacket);

static aErr sPacketAddByte(aBD* pBD,
			   aBDPacket* pPacket,
			   const int byte);

static aErr sUpdateStatus (aBD* pBD,
			   const char* pText);

			   

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBD_ParseErr
 */

aErr sBD_ParseErr(tkError error,
		  const unsigned int nLine,
		  const unsigned int nColumn,
		  const unsigned int nData,
		  const char* data[],
		  void* errProcRef)
{
  aErr bdErr = aErrNone;
/*  aBD* pBD = (aBD*)errProcRef; */
  
  return bdErr;

} /* sBD_ParseErr */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStart
 */

aErr sStart(aXMLNodeRef node,
		   const char* pKey,
		   void* vpRef)
{
  aErr bdErr = aErrNone;
  aBD* pBD = (aBD*)vpRef;

  /* turn on the importing flag */
  if (!aStringCompare(pKey, "BRAINDUMP")) {
    pBD->flags |= kBDImporting;
  } else if (!aStringCompare(pKey, "CUP")) {
    if (pBD->pCurrent) {
      /* report the nested object error */
      if (pBD->status)
        aStream_WriteLine(pBD->ioRef, pBD->status, 
        		  "Objects can not be nested", NULL);
    } else {
      aAssert(!pBD->pCurrent);
      bdErr = sBuildObject(pBD, kBDObjectCup, &pBD->pCurrent);
    }
  } else if (!aStringCompare(pKey, "BATCH")) {
    if (pBD->pCurrent) {
      /* report the nested object error */
      if (pBD->status)
        aStream_WriteLine(pBD->ioRef, pBD->status, 
        		  "Objects can not be nested", NULL);
    } else {
      aAssert(!pBD->pCurrent);
      bdErr = sBuildObject(pBD, kBDObjectBatch, &pBD->pCurrent);
    }
  }

  return bdErr;

} /* sStart */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sContent
 */

aErr sContent(aXMLNodeRef node,
	      const char* pKey,
	      const aToken* pToken,
	      void* vpRef)
{
  aErr bdErr = aErrNone;
  aBD* pBD = (aBD*)vpRef;

  if (pBD->pCurrent) { 
    if (!aStringCompare(pKey, "NAME")) {
      if (pToken->eType != tkString) {
        /* report error for string expected */
      } else {
        if (pBD->pCurrent->setName)
          bdErr = pBD->pCurrent->setName(pBD, pToken->v.string);
      }
    } else if (!aStringCompare(pKey, "SLOT")) {
      if (pToken->eType != tkInt) {
        /* report error for string expected */
      } else {
        if (pBD->pCurrent->setSlot)
          bdErr = pBD->pCurrent->setSlot(pBD, 
          			(unsigned char)pToken->v.integer);
      }
    } else if (!aStringCompare(pKey, "MODULE")) {
      if (pToken->eType != tkInt) {
        /* report error for string expected */
      } else {
        if (pBD->pCurrent->setSlot)
          bdErr = pBD->pCurrent->setModule(pBD, 
          			(unsigned char)pToken->v.integer);
      }
    }
  }

  return bdErr;

} /* sContent */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sEnd
 */

aErr sEnd(aXMLNodeRef node,
	  aXMLNodeRef parent,
	  const char* pKey,
	  void* vpRef)
{
  aErr bdErr = aErrNone;
  aBD* pBD = (aBD*)vpRef;

  /* turn off the importing flag */
  if (!aStringCompare(pKey, "BRAINDUMP")) {
    pBD->flags &= ~kBDImporting;

  /* cup files and associated data are added to the 
   * object list */
  } else if (!aStringCompare(pKey, "CUP")) {
    if (pBD->pCurrent->o.cup.fileBuffer 
        && (pBD->pCurrent->o.cup.nSlot != 0xFF)
        && (pBD->pCurrent->o.cup.nModule != 0xFF)) {
      bdErr = sAddObject(pBD, pBD->pCurrent);
      pBD->pCurrent = NULL;
    }
   
  /* batches don't get added to the object list as the 
   * component packets contained in the batch have already
   * been added */
  } else if (!aStringCompare(pKey, "BATCH")) {
    aMemPool_Free(pBD->ioRef, pBD->pObjectPool, 
    		  pBD->pCurrent, &bdErr);
    pBD->pCurrent = NULL;
  }

  return bdErr;

} /* sEnd */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCupSetName
 */

aErr sCupSetName(aBD* pBD,
		 const char* pName)
{
  aErr bdErr = aErrNone;
  aStreamRef cupFile = NULL;

  aVALIDBD(pBD);
  
  /* read in the cup file */
  if (bdErr == aErrNone) {
    aStream_CreateFileInput(pBD->ioRef,
  			    pName,
  			    aFileAreaObject,
  			    &cupFile,
  			    &bdErr);
  }
  
  if (bdErr == aErrNotFound) {
    pBD->nErrors++;
    if (pBD->status) {
      char line[200];
      aStringCopySafe(line, 200, "cup file \"");
      aStringCatSafe(line, 200, pName);
      aStringCatSafe(line, 200, "\" not found");
      aStream_WriteLine(pBD->ioRef, pBD->status, line, &bdErr);
    }
    bdErr = aErrNone;
  } else {
    aStreamBuffer_Create(pBD->ioRef, 128, 
    			 &pBD->pCurrent->o.cup.fileBuffer, &bdErr);
    
    if (bdErr == aErrNone)
      aStream_Flush(pBD->ioRef, cupFile, 
      			  pBD->pCurrent->o.cup.fileBuffer, &bdErr);
  }

  if (cupFile)
    aStream_Destroy(pBD->ioRef, cupFile, NULL);

  return bdErr;

} /* sCupSetName */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCupSetSlot
 */

aErr sCupSetSlot(aBD* pBD,
		 const unsigned char nSlot)
{
  aErr bdErr = aErrNone;

  aVALIDBD(pBD);
  
  if (bdErr == aErrNone) {
    /* range check the input */
    
    aAssert(pBD->pCurrent->nType == kBDObjectCup);
    pBD->pCurrent->o.cup.nSlot = nSlot;
  }

  return bdErr;

} /* sCupSetSlot */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCupSetModule
 */

aErr sCupSetModule(aBD* pBD,
		   const unsigned char nModule)
{
  aErr bdErr = aErrNone;

  aVALIDBD(pBD);
  
  if (bdErr == aErrNone) {
    /* range check the input */
    
    aAssert(pBD->pCurrent->nType == kBDObjectCup);
    pBD->pCurrent->o.cup.nModule = nModule;
  }

  return bdErr;

} /* sCupSetModule */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCupCleanup
 */

aErr sCupCleanup(aBD* pBD,
		 aBDObject* pCup)
{
  aErr bdErr = aErrNone;
  
  aAssert(pCup->nType == kBDObjectCup);

  if (pCup->o.cup.fileBuffer) {
    aStream_Destroy(pBD->ioRef, pCup->o.cup.fileBuffer, &bdErr);
    pCup->o.cup.fileBuffer = NULL;
  }

  if (pCup->o.cup.pCopy) {
    aMemFree(pCup->o.cup.pCopy);
    pCup->o.cup.pCopy = NULL;
  }

  return bdErr;

} /* sCupCleanup */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCupWrite
 */

aErr sCupWrite(aBD* pBD,
	       aBDObject* pCup,
	       aStreamRef destStream)
{
  aErr bdErr = aErrNone;
  aMemSize size;
  char buf[2];
  
  aAssert(pCup->nType == kBDObjectCup);

  /* write the module number */
  if (bdErr == aErrNone)
    aStream_Write(pBD->ioRef, destStream, 
    		  (char*)&pCup->o.cup.nModule, 1, &bdErr);

  /* write the slot number */
  if (bdErr == aErrNone)
    aStream_Write(pBD->ioRef, destStream, 
    		  (char*)&pCup->o.cup.nSlot, 1, &bdErr);

  /* write the size of the cup file */
  if (bdErr == aErrNone) {
    aAssert(pCup->o.cup.fileBuffer);
    aStreamBuffer_Get(pBD->ioRef,
    		      pCup->o.cup.fileBuffer,
    		      &size,
    		      NULL,
    		      &bdErr);
    aUtil_StoreShort(buf, (short)size);
    aStream_Write(pBD->ioRef, destStream, buf, 2, &bdErr);
  }
  
  /* then, write out the buffer */
  if (bdErr == aErrNone)
    aStreamBuffer_Flush(pBD->ioRef, pCup->o.cup.fileBuffer, 
    		        destStream, &bdErr);

  return bdErr;

} /* sCupWrite */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCupRead
 */

aErr sCupRead(aBD* pBD,
	      aBDObject* pCup,
	      aStreamRef dumpStream)
{
  aErr bdErr = aErrNone;
  aMemSize size = 0;
  char buf[2];
  int n = 0;
  
  aAssert(pCup->nType == kBDObjectCup);

  /* read the module number */
  if (bdErr == aErrNone)
    aStream_Read(pBD->ioRef, dumpStream, 
    		  (char*)&pCup->o.cup.nModule, 1, &bdErr);

  /* write the slot number */
  if (bdErr == aErrNone)
    aStream_Read(pBD->ioRef, dumpStream, 
    		  (char*)&pCup->o.cup.nSlot, 1, &bdErr);

  /* read the size of the cup file */
  if (bdErr == aErrNone) {
    aStream_Read(pBD->ioRef, dumpStream, buf, 2, &bdErr);
    if (bdErr == aErrNone)
      size = aUtil_RetrieveShort(buf);
  }
  
  /* allocate space for a copy */
  pCup->o.cup.pCopy = NULL;
  pCup->o.cup.pCopy = (char*)aMemAlloc(size);
  pCup->o.cup.nSize = (unsigned short)size;
  
  /* then, read the cup into buffer (and make a copy) */
  if (bdErr == aErrNone) {
    aStreamBuffer_Create(pBD->ioRef, 128,
    			 &pCup->o.cup.fileBuffer, &bdErr);
    while (size && (bdErr == aErrNone)) {
      aStream_Read(pBD->ioRef, dumpStream, buf, 1, &bdErr);
      pCup->o.cup.pCopy[n++] = buf[0];
      if (bdErr == aErrNone)
        aStream_Write(pBD->ioRef, pCup->o.cup.fileBuffer, buf, 
        	      1, &bdErr);
      size--;
    }
  }

  return bdErr;

} /* sCupRead */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCupDump
 */

aErr sCupDump(aBD* pBD,
	      aBDObject* pCup,
	      aStemLib stemRef)
{
  aErr bdErr = aErrNone;
  aStreamRef slotStream = NULL;

  aAssert(pCup->nType == kBDObjectCup);
  aAssert(pBD->stemRef);

  if (bdErr == aErrNone) {

    char buff[40];
    char num[10];
    int i;

    aStringCopySafe(buff, 40, "CUP file: ");
    aStringFromInt(num, pCup->o.cup.nModule);
    aStringCatSafe(buff, 40, num);
    aStringCatSafe(buff, 40, " ");
    aStringFromInt(num, pCup->o.cup.nSlot);
    aStringCatSafe(buff, 40, num);

    bdErr = sUpdateStatus(pBD, buff);

    /* this short loop gives some time for the UI to update */
    for (i = 0; i < 5; i++) {
      sUpdateStatus(pBD, NULL);
      aIO_MSSleep(pBD->ioRef, 1, NULL);
    }
  }

  /* try to build a connection to the slot on the module */
  if (bdErr == aErrNone) {
    aStem_CreateTEAFileOutput(pBD->stemRef,
    			      pCup->o.cup.nModule,
    			      pCup->o.cup.nSlot,
    			      &slotStream, 
    			      &bdErr);
    if ((bdErr != aErrNone) && pBD->status)
      aStream_WriteLine(pBD->ioRef,
      			pBD->status, 
      			"slot initialization failed",
      			NULL);
  }

  /* do the actual loading */
  if (bdErr == aErrNone) {
    aErr copyErr = aErrNone;
    aStreamBuffer_Flush(pBD->ioRef,
    			pCup->o.cup.fileBuffer,
    			slotStream,
    			&copyErr);
    /* now determine whether we succeeded or not */
    if (copyErr == aErrNone) {
      if (pBD->status)
        aStream_WriteLine(pBD->ioRef,
      			  pBD->status, 
      			  "slot loaded",
      			  NULL);
    } else {
      if (pBD->status)
        aStream_WriteLine(pBD->ioRef,
      			  pBD->status, 
      			  "slot loading failed",
      			  NULL);
      bdErr = aErrIO;
    }
  }


  /* clean up the slot stream either way if it was opened */
  /* final checksum could have been bad, that error is caught here */
  if (slotStream != NULL) {
    aStream_Destroy(aStreamLibRef(slotStream), slotStream, &bdErr);
    slotStream = NULL;
  }

  if (bdErr == aErrWrite) {
    sUpdateStatus(pBD, "CUP File Checksum Error");
  }
  if (bdErr == aErrTimeout) {
    sUpdateStatus(pBD, "Timeout Error");
  }

  /* read back what was loaded */
  if (bdErr == aErrNone) {
    aErr readErr = aErrNone;
    int i;
    int k = 0;
    int m = 0;
    int address = 0;
    aPacketRef packet;
    char data[aSTEMMAXPACKETBYTES];
    unsigned char ucaddress;
    unsigned char uclength;

    sUpdateStatus(pBD, "Verifying...");

    /* this short loop gives some time for the UI to update */
    for (i = 0; i < 5; i++) {
      sUpdateStatus(pBD, NULL);
      aIO_MSSleep(pBD->ioRef, 1, NULL);
    }

    /* read EEPROM, write correction if error found */
    /* CUP files start at (5K + 2) + slot*1K */ 
    address = (pCup->o.cup.nSlot * 1024) + 5122;
    
    while ((k < pCup->o.cup.nSize) && (readErr == aErrNone)) {

      m = pCup->o.cup.nSize - k;
      if (m > aBD_READINC) m = aBD_READINC;

      data[0] = cmdMEM_RD;
      data[1] = (char)(address >> 8);
      data[2] = (char)(address & 0xFF);
      data[3] = (char)m;
      if (readErr == aErrNone)
        aPacket_Create(pBD->stemRef,
    		   (unsigned char)pCup->o.cup.nModule,
    		   (unsigned char)4,
    		   data,
    		   &packet,
    		   &readErr);

      if (readErr == aErrNone)
        aStem_SendPacket(pBD->stemRef,
    		     packet,
    		     &readErr);

      if (readErr == aErrNone)
        aStem_GetPacket(pBD->stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdMEM_VAL,
      		         500, 
      		         &packet, 
      		         &readErr);

      if (readErr == aErrNone)
        aPacket_GetData(pBD->stemRef, 
        		    packet, 
        		    &ucaddress,
      		            &uclength, 
      		            data, 
      		            &readErr);

      if (readErr == aErrNone)
        aPacket_Destroy(pBD->stemRef, packet, NULL);

      if (readErr == aErrNone) {
        int nn;
        for (nn = 0; nn < m; nn++) {
          if (pCup->o.cup.pCopy[k + nn] != data[1 + nn]) {

            /* correct any screwed up bytes */
            aPacketRef xpacket;
            /* sUpdateStatus(pBD, "rewriting byte"); */
            data[0] = cmdMEM_WR;
            data[1] = (char)((address + nn) >> 8);
            data[2] = (char)((address + nn) & 0xFF);
            data[3] = (char)(pCup->o.cup.pCopy[k + nn]);
            if (readErr == aErrNone)
              aPacket_Create(
                pBD->stemRef,
                (unsigned char)pCup->o.cup.nModule,
                (unsigned char)4,
    		data,
    		&xpacket,
    		&readErr);

            if (readErr == aErrNone)
              aStem_SendPacket(pBD->stemRef,
    		     xpacket,
    		     &readErr);
          }
        }
      }

      k += aBD_READINC;
      address += aBD_READINC;
    }

    if (readErr != aErrNone) {
      if (readErr == aErrTimeout)
        sUpdateStatus(pBD, "Timeout Error.");
      sUpdateStatus(pBD, "Verification failed.");
      bdErr = aErrRead;
    }
  }

  return bdErr;

} /* sCupDump */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBatchSetName
 */

aErr sBatchSetName(aBD* pBD,
		   const char* pName)
{
  aErr bdErr = aErrNone;
  aStreamRef batchFile = NULL;

  aVALIDBD(pBD);

  /* read in the cup file */
  if (bdErr == aErrNone) {
    aStream_CreateFileInput(pBD->ioRef,
  			    pName,
  			    aFileAreaUser,
  			    &batchFile,
  			    &bdErr);
  }

  if (bdErr == aErrNotFound) {
    pBD->nErrors++;
    if (pBD->status) {
      char line[200];
      aStringCopySafe(line, 200, "batch file \"");
      aStringCatSafe(line, 200, pName);
      aStringCatSafe(line, 200, "\" not found");
      aStream_WriteLine(pBD->ioRef, pBD->status, line, &bdErr);
    }
    bdErr = aErrNone;
  } else {

    /* read in the batch and build packets as you go */
    aErr readErr = aErrNone;
    char line[aBD_MAXINPUTLINE];
    while ((readErr == aErrNone) && (bdErr == aErrNone)) {
      aStream_ReadLine(pBD->ioRef, batchFile, line, 
      		       aBD_MAXINPUTLINE, &readErr);
      if (readErr == aErrNone) {
        aStreamRef lineStream;
        aTokenizerRef tokenizer;
        aBDObject* pPO = NULL;
        aToken* pToken;
        aBool bHasBytes = aFalse;
        bdErr = aStream_Create_TextLine_Input(pBD->ioRef,
        				      line,
        				      &lineStream);
        /* build a packet object */
        if (bdErr == aErrNone)
          bdErr = sBuildObject(pBD, kBDObjectPacket, &pPO);
          
        /* build a tokenizer for the text line */
        if (bdErr == aErrNone)
          aTokenizer_Create(pBD->ioRef, 
    		            lineStream, 
    		            pName,
    		            aFileAreaUser,
    		            sBD_ParseErr,
    		            pBD, 
    		            &tokenizer, 
    		            &bdErr);

        /* parse the text line into the packet */
        if (bdErr == aErrNone)
          bdErr = sPacketInit(&pPO->o.packet);

        /* get packets up till the packet is full */
        do {
          if (bdErr != aErrNone) break;
          pToken = NULL;
          if (!aTokenizer_Next(pBD->ioRef, tokenizer, 
          		       &pToken, NULL)) {
            if (pToken->eType == tkInt) {
              sPacketAddByte(pBD, &pPO->o.packet, pToken->v.integer);
              bHasBytes = aTrue;
            } else {
            }
            
            if (bdErr == aErrNone)
              aTokenizer_Dispose(pBD->ioRef, tokenizer,
            		         pToken, NULL);
          }
        } while ((bdErr == aErrNone) && pToken);

        if (bdErr == aErrNone)
          aTokenizer_Destroy(pBD->ioRef, tokenizer, &bdErr);

        /* add the packet to the list */
        if (bdErr == aErrNone) {
          if (bHasBytes)
            bdErr = sAddObject(pBD, pPO);
          else
            bdErr = sDeleteObject(pBD, pPO);
        }

      } else if (readErr != aErrEOF) {
        bdErr = readErr;
      }
    } /* while */
  }

  if (batchFile)
    aStream_Destroy(pBD->ioRef, batchFile, NULL);

  return bdErr;

} /* sBatchSetName */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPacketWrite
 */

aErr sPacketWrite(aBD* pBD,
	       aBDObject* pPacket,
	       aStreamRef destStream)
{
  aErr bdErr = aErrNone;
  
  aAssert(pPacket->nType == kBDObjectPacket);

  /* write the module address */
  if (bdErr == aErrNone)
    aStream_Write(pBD->ioRef, destStream, 
    		  (char*)&pPacket->o.packet.nAddress, 1, &bdErr);

  /* write the packet length */
  if (bdErr == aErrNone)
    aStream_Write(pBD->ioRef, destStream, 
    		  (char*)&pPacket->o.packet.nLength, 1, &bdErr);

  /* write the data */
  if (bdErr == aErrNone) {
    aStream_Write(pBD->ioRef, destStream, pPacket->o.packet.data, 
    		  pPacket->o.packet.nLength, &bdErr);
  }

  return bdErr;

} /* sPacketWrite */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPacketRead
 */

aErr sPacketRead(aBD* pBD,
	      aBDObject* pPacket,
	      aStreamRef dumpStream)
{
  aErr bdErr = aErrNone;
  
  aAssert(pPacket->nType == kBDObjectPacket);

  /* read the module address */
  if (bdErr == aErrNone)
    aStream_Read(pBD->ioRef, dumpStream, 
    		  (char*)&pPacket->o.packet.nAddress, 1, &bdErr);

  /* read in the packet length */
  if (bdErr == aErrNone)
    aStream_Read(pBD->ioRef, dumpStream, 
    		  (char*)&pPacket->o.packet.nLength, 1, &bdErr);

  /* read the data */
  if (bdErr == aErrNone)
    aStream_Read(pBD->ioRef, dumpStream, pPacket->o.packet.data, 
    		 pPacket->o.packet.nLength, &bdErr);

  return bdErr;

} /* sPacketRead */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPacketDump
 */

aErr sPacketDump(aBD* pBD,
	      aBDObject* pPacket,
	      aStemLib stemRef)
{
  aErr bdErr = aErrNone;
  aPacketRef packet;

  aAssert(pPacket->nType == kBDObjectPacket);
  aAssert(pBD->stemRef);

  /* since GP has lowest IIC baud rate (for speech option)
   * batch packets could pile up and overflow the IIC queue
   * this delay keeps long batch files from overflowing queue */
  aIO_MSSleep(pBD->ioRef, 5, NULL);
  
  /* pseudo time slice -- just get any extra packets and toss them
   * which allows us to catch any queued up heartbeats */
  aStem_GetPacket(pBD->stemRef, NULL, NULL, 
    		  0, &packet, &bdErr);
  if (bdErr == aErrNone)
    aPacket_Destroy(pBD->stemRef, packet, &bdErr);
  else if ((bdErr == aErrNotFound) || (bdErr == aErrTimeout))
    bdErr = aErrNone;
  

  /* build a stem packet with the data */
  if (bdErr == aErrNone)
    aPacket_Create(pBD->stemRef,
    		   pPacket->o.packet.nAddress,
    		   pPacket->o.packet.nLength,
    		   pPacket->o.packet.data,
    		   &packet,
    		   &bdErr);

  /* send the packet */
  if (bdErr == aErrNone)
    aStem_SendPacket(pBD->stemRef,
    		     packet,
    		     &bdErr);
  return bdErr;

} /* sPacketDump */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sBuildObject
 */

aErr sBuildObject(aBD* pBD,
	          const short nType,
	          aBDObject** ppObject)
{
  aErr bdErr = aErrNone;
  void* pMemory = NULL;
  aBDObject* pObject = NULL;

  /* get the storage for the object */
  if (bdErr == aErrNone)
    aMemPool_Alloc(pBD->ioRef, pBD->pObjectPool, &pMemory, &bdErr);

  /* zero it out to start with */
  if (bdErr == aErrNone) {
    pObject = (aBDObject*)pMemory;
    aBZero(pObject, sizeof(aBDObject));
    pObject->nType = nType;
  }

  switch (nType) {
  
  case kBDObjectCup:
    pObject->setName = sCupSetName;
    pObject->setSlot = sCupSetSlot;
    pObject->setModule = sCupSetModule;
    pObject->cleanup = sCupCleanup;
    pObject->write = sCupWrite;
    pObject->read = sCupRead;
    pObject->dump = sCupDump;
    pObject->o.cup.nSlot = 0xFF;
    pObject->o.cup.nModule = 0xFF;
    break;

  case kBDObjectBatch:
    pObject->setName = sBatchSetName;
    break;

  case kBDObjectPacket:
    pObject->write = sPacketWrite;
    pObject->read = sPacketRead;
    pObject->dump = sPacketDump;
    break;

  default:
    bdErr = aErrUnknown;
    break;

  } /* switch */

  if (bdErr == aErrNone)
    *ppObject = pObject;
  else if (pObject)
    aMemPool_Free(pBD->ioRef, pBD->pObjectPool, pObject, NULL);

  return bdErr;

} /* sBuildObject */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAddObject
 *
 * Adds objects at the end of the list to preserve order.
 */

aErr sAddObject(aBD* pBD,
		aBDObject* pObject)
{
  aErr bdErr = aErrNone;
  
  aVALIDBD(pBD);
  
  if (bdErr == aErrNone) {
    if (!pBD->pObjects)
      pBD->pObjects = pObject;
    if (pBD->pTail)
      pBD->pTail->pNext = pObject;
    pBD->pTail = pObject;
  }

  return bdErr;
  
} /* sAddObject */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sDeleteObject
 *
 * Deletes the object and its associated resources.
 */

aErr sDeleteObject(aBD* pBD,
		   aBDObject* pObject)
{
  aErr bdErr = aErrNone;
  
  aVALIDBD(pBD);
  
  if (bdErr == aErrNone) {
    if (pObject->cleanup)
      bdErr = pObject->cleanup(pBD, pObject);
    if (bdErr == aErrNone)
      aMemPool_Free(pBD->ioRef, pBD->pObjectPool, pObject, &bdErr);
  }

  return bdErr;
  
} /* sDeleteObject */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPacketInit
 */

aErr sPacketInit(aBDPacket* pPacket)
{
  aErr bdErr = aErrNone;

  pPacket->fStatus = 0;

  return bdErr;

} /* sPacketInit */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPacketAddByte
 */

aErr sPacketAddByte(aBD* pBD,
		    aBDPacket* pPacket,
		    const int byte)
{
  aErr bdErr = aErrNone;

  /* start with an address */
  if (!pPacket->fStatus & kBDPktHasAddress) {
    if (byte & 0x01) {
      bdErr = aErrParam;
    } else if ((byte < 2) || (byte > 254)) {
      bdErr = aErrRange;
    } else {
      pPacket->fStatus |= kBDPktHasAddress;
      pPacket->nAddress = (unsigned char)byte;
    }
  }

  /* the data */
  else {
    if (pPacket->nBytes >= aSTEMMAXPACKETBYTES) {
      bdErr = aErrRange;
    } else {
      pPacket->data[pPacket->nLength++] = (char)byte;
    }
  }

  return bdErr;

} /* sPacketAddByte */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sUpdateStatus
 *
 * Sends the status to both the UI (if proc is present) and the 
 * StemDebug file.
 */

aErr sUpdateStatus (
  aBD* pBD,
  const char* pText
)
{
  aErr err = aErrNone;

  if ((err == aErrNone) && (pBD->statusProc))
    err = pBD->statusProc(pText, pBD->statusRef);
    
  if (err == aErrNone)
    aStem_DebugLine(pBD->stemRef, pText, &err);
  
  return err;

} /* sUpdateStatus */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBD_Create
 */

aErr aBD_Create(aIOLib ioRef,
		aStemLib stemRef,
		aBDStatusProc statusProc,
		void* statusRef,
		aBDRef* pBDRef)
{
  aErr bdErr = aErrNone;
  aBD* pBD = NULL;
  
  if (!pBDRef)
    bdErr = aErrParam;

  if (bdErr == aErrNone) {
    pBD = (aBD*)aMemAlloc(sizeof(aBD));
    if (pBD) {
      aBZero(pBD, sizeof(aBD));
      pBD->check = aBDCHECK;
    } else 
      bdErr = aErrMemory;
  }

  if (bdErr == aErrNone) {
    pBD->ioRef = ioRef;
    pBD->stemRef = stemRef;
    pBD->statusProc = statusProc;
    pBD->statusRef = statusRef;
    aMemPool_Create(ioRef, sizeof(aBDObject), 8, 
    		    &pBD->pObjectPool, &bdErr);
  }

  if (bdErr == aErrNone)
    *pBDRef = (aBDRef)pBD;

  return bdErr;

} /* aBD_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBD_CheckLock
 */

aErr aBD_CheckLock(
  aBDRef bdRef,
  unsigned char module,
  unsigned int address)
{
  aErr bdErr = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];
  unsigned char ucaddress;
  unsigned char uclength;
  char cNewByte = 0;
  char cCheckByte = 0;
  aBD* pBD = (aBD*)bdRef;
  char addrH = (char)((address >> 8) & 0xFF);
  char addrL = (char)(address & 0xFF);


  aVALIDBD(bdRef);


  /* (1) read check position */
  data[0] = cmdMEM_RD;
  data[1] = addrH;
  data[2] = addrL;
  data[3] = 1;
  if (bdErr == aErrNone)
    aPacket_Create(pBD->stemRef,
    		   (unsigned char)module,
    		   (unsigned char)4,
    		   data,
    		   &packet,
    		   &bdErr);

  /* send the packet */
  if (bdErr == aErrNone)
    aStem_SendPacket(pBD->stemRef,
    		     packet,
    		     &bdErr);

  if (bdErr == aErrNone) {
    if (!aStem_GetPacket(pBD->stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdMEM_VAL,
      		         500, 
      		         &packet, 
      		         &bdErr)
        && !aPacket_GetData(pBD->stemRef, 
        		    packet, 
        		    &ucaddress,
      		            &uclength, 
      		            data, 
      		            &bdErr)) {

      /* stash check byte */
      /* toggle new byte to write back */
      int q;
      cCheckByte = data[1];
      q = cCheckByte;
      q = ~q;
      cNewByte = (char)q;
      aPacket_Destroy(pBD->stemRef, packet, NULL);
    }
  }

  /* (2) write toggle byte */
  data[0] = cmdMEM_WR;
  data[1] = addrH;
  data[2] = addrL;
  data[3] = cNewByte;
  if (bdErr == aErrNone)
    aPacket_Create(pBD->stemRef,
    		   (unsigned char)module,
    		   (unsigned char)4,
    		   data,
    		   &packet,
    		   &bdErr);

  /* send the packet */
  if (bdErr == aErrNone)
    aStem_SendPacket(pBD->stemRef,
    		     packet,
    		     &bdErr);



  /* (3) read check position again */
  data[0] = cmdMEM_RD;
  data[1] = addrH;
  data[2] = addrL;
  data[3] = 1;
  if (bdErr == aErrNone)
    aPacket_Create(pBD->stemRef,
    		   (unsigned char)module,
    		   (unsigned char)4,
    		   data,
    		   &packet,
    		   &bdErr);

  /* send the packet */
  if (bdErr == aErrNone)
    aStem_SendPacket(pBD->stemRef,
    		     packet,
    		     &bdErr);

  if (bdErr == aErrNone) {
    if (!aStem_GetPacket(pBD->stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdMEM_VAL,
      		         500, 
      		         &packet, 
      		         &bdErr)
        && !aPacket_GetData(pBD->stemRef, 
        		    packet, 
        		    &ucaddress,
      		            &uclength, 
      		            data, 
      		            &bdErr)) {

      /* un-toggle the new byte */
      /* see if it matches check byte */
      /* (write failure will be noted by mismatch) */
      int q;
      cNewByte = data[1];
      q = cNewByte;
      q = ~q;
      cNewByte = (char)q;
      if (cNewByte != cCheckByte) {
        bdErr = aErrWrite;
        sUpdateStatus (pBD, "Check EEPROM lock");
        if (pBD->status)
          aStream_WriteLine(pBD->ioRef, pBD->status, "Write failure, check EEPROM lock", NULL);
      }
      aPacket_Destroy(pBD->stemRef, packet, NULL);
    }
  }

  if (bdErr == aErrNone) {
  }

  return bdErr;

} /* aBD_CheckLock */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBD_Destroy
 */

aErr aBD_Destroy(aBDRef bdRef)
{
  aErr bdErr = aErrNone;
  aBD* pBD = (aBD*)bdRef;

  aVALIDBD(bdRef);

  /* clean up the object list */
  if (bdErr == aErrNone) {
    aBDObject* pTemp;
    while (pBD->pObjects) {
      pTemp = pBD->pObjects;
      pBD->pObjects = pTemp->pNext;
      bdErr = sDeleteObject(pBD, pTemp);
    }
  }

  /* clean up the object memory pool storage */
  if (bdErr == aErrNone)
    aMemPool_Destroy(pBD->ioRef, pBD->pObjectPool, &bdErr);

  if (bdErr == aErrNone) {
    pBD->check = 0;
    aMemFree(pBD);
  }

  return bdErr;

} /* aBD_Destroy */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBD_ReadXML
 *
 * Imports xml description file contents into a brain dump.
 */

aErr aBD_ReadXML(aBDRef bdRef, 
		 aStreamRef xmlStream,
		 aStreamRef status)
{
  aErr bdErr = aErrNone;
  aXMLRef xml;
  aBD* pBD = (aBD*)bdRef;

  aVALIDBD(bdRef);
  if ((bdErr == aErrNone) && !xmlStream)
    bdErr = aErrParam;

  /* set up the status stream for processing */
  if (bdErr == aErrNone)
    pBD->status = status;
  
  if (bdErr == aErrNone) {
    aXML_Create(pBD->ioRef,
    		xmlStream,
    		sBD_ParseErr,
    		pBD,
    		&xml,
    		&bdErr);
  }

  /* set up the XML import and run it */
  if (bdErr == aErrNone) {
    aXMLCallbacks cb;
    cb.handleStart = sStart;
    cb.handleContent = sContent;
    cb.handleEnd = sEnd;
    cb.ref = pBD;
    pBD->status = status;
    pBD->nErrors = 0;
    if (status)
      aStream_WriteLine(pBD->ioRef, status, "Importing XML", NULL);
    aXML_Traverse(pBD->ioRef, xml, &cb, &bdErr);
  }

  /* clean up the XML object */
  if (bdErr == aErrNone)
    aXML_Destroy(pBD->ioRef, xml, &bdErr);

  /* report the import status */
  if ((bdErr == aErrNone) && pBD->status) {
    if (pBD->nErrors) {
      char line[100];
      aSNPRINTF(line, 100, "There Were %d Import Errors.", pBD->nErrors);
      aStream_WriteLine(pBD->ioRef, status, line, NULL);
    } else {
      aStream_WriteLine(pBD->ioRef, status, "No Import Errors", NULL);
    }
  }

  /* reset the status stream to NULL when done */
  if (bdErr == aErrNone)
    pBD->status = NULL;

  return bdErr;

} /* aBD_ReadXML */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBD_Write
 */

aErr aBD_Write(aBDRef bdRef, 
	       aStreamRef destStream)
{
  aErr bdErr = aErrNone;
  aBD* pBD = (aBD*)bdRef;
  char buf[2];
  aBDObject* pObject;

  aVALIDBD(bdRef);
  if ((bdErr == aErrNone) && !destStream)
    bdErr = aErrParam;

  /* write the signature */
  if (bdErr == aErrNone)
    aStream_Write(pBD->ioRef, destStream, "ACRO", 4, &bdErr);
  
  /* write the version number */
  if (bdErr == aErrNone) {
    aUtil_StoreShort(buf, aBDVERSION);
    aStream_Write(pBD->ioRef, destStream, buf, 2, &bdErr);
  }

  /* write the number of objects */
  if (bdErr == aErrNone) {
    short num = 0;
    pObject = pBD->pObjects;
    while (pObject) {
      num++;
      pObject = pObject->pNext;
    }
    aUtil_StoreShort(buf, num);
    aStream_Write(pBD->ioRef, destStream, buf, 2, &bdErr);
  }

  /* finally, write the objects */
  pObject = pBD->pObjects;
  while (pObject && (bdErr == aErrNone)) {
    /* write the object type */
    aUtil_StoreShort(buf, pObject->nType);
    aStream_Write(pBD->ioRef, destStream, buf, 2, &bdErr);
    aAssert(pObject->write);
    bdErr = pObject->write(pBD, pObject, destStream);
    pObject = pObject->pNext;
  }

  return bdErr;

} /* aBD_Write */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBD_Read
 */

aErr aBD_Read(aBDRef bdRef, 
	      aStreamRef dumpStream)
{
  aErr bdErr = aErrNone;
  aBD* pBD = (aBD*)bdRef;
  char buf[5];
  short num = 0;

  aVALIDBD(bdRef);
  if ((bdErr == aErrNone) && !dumpStream)
    bdErr = aErrParam;

  /* read the signature */
  if (bdErr == aErrNone) {
    aStream_Read(pBD->ioRef, dumpStream, buf, 4, &bdErr);
    if (bdErr == aErrNone) {
      buf[4] = 0;
      if (aStringCompare(buf, "ACRO"))
        bdErr = aErrIO;
    }
  }

  /* read the version number */
  if (bdErr == aErrNone) {
    aStream_Read(pBD->ioRef, dumpStream, buf, 2, &bdErr);
    if (bdErr == aErrNone) {
      short version = aUtil_RetrieveShort(buf);
      if (version > aBDVERSION)
        bdErr = aErrVersion;
    }
  }

  /* read the number of objects */
  if (bdErr == aErrNone) {
    aStream_Read(pBD->ioRef, dumpStream, buf, 2, &bdErr);
    num = aUtil_RetrieveShort(buf);
  }

  /* finally, read in the objects */
  while (num && (bdErr == aErrNone)) {
    aStream_Read(pBD->ioRef, dumpStream, buf, 2, &bdErr);
    if (bdErr == aErrNone) {
      short type = aUtil_RetrieveShort(buf);
      aBDObject* pObject = NULL;
      bdErr = sBuildObject(pBD, type, &pObject);
      if (bdErr == aErrNone) {
        aAssert(pObject->read);
        bdErr = pObject->read(pBD, pObject, dumpStream);
      }
      if (bdErr == aErrNone)
        bdErr = sAddObject(pBD, pObject);
    }
    num--;
  }

  return bdErr;

} /* aBD_Read */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aBD_Dump
 */

aErr aBD_Dump(aBDRef bdRef)
{
  aErr bdErr = aErrNone;
  aBD* pBD = (aBD*)bdRef;
  unsigned char ucLinkAddr;
  aBool b2Way = aFalse;

  aVALIDBD(bdRef);

  /* are we live? */
  bdErr = aModuleUtil_CheckForModule(pBD->stemRef, &ucLinkAddr);

  /* try to get module's attention */  
  if (bdErr == aErrNone) {
    sUpdateStatus(pBD, "module found");
    bdErr = aModuleUtil_ForceLink(pBD->stemRef, ucLinkAddr);
  }

  /* check for 2-way communication */
  if (bdErr == aErrNone) {
    b2Way = aModuleUtil_EnsureModule(pBD->stemRef, ucLinkAddr);
    if (b2Way) {
      sUpdateStatus(pBD, "two-way link established");
    } else {
      bdErr = aErrConfiguration;
    }
  }

  if (bdErr == aErrNone) {  
    /* walk the list of objects and dump them to the brainstem link */  
    if (bdErr == aErrNone) {
      aBDObject* pObject = pBD->pObjects;
      while ((bdErr == aErrNone) && pObject) {
        if (pObject->dump)
          bdErr = pObject->dump(pBD, pObject, pBD->stemRef);
        pObject = pObject->pNext;
      }
    }
  }

  return bdErr;

} /* aBD_Dump */

