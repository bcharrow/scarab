/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGDPNG.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent graphics  */
/*		layer.						   */
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

#include "aGD.h"
#include "aGDOffscreen.h"
#include "aUtil.h"


typedef struct aPNGChunkStream {
  aIOLib	ioRef;
  aStreamRef	buffer;
} aPNGChunkStream;
static aErr sPNGChunkStream_Init (
  aIOLib ioRef,
  const char* type,
  aPNGChunkStream* pChunk
);
static aErr sPNGChunkStream_Add (
  aPNGChunkStream* pChunk,
  char* pData,
  unsigned long length
);
static aErr sPNGChunkStream_Finish ( 
  aPNGChunkStream* pChunk,
  aStreamRef output
);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPNGChunkStream_Init
 */

aErr sPNGChunkStream_Init (
  aIOLib ioRef,
  const char* type,
  aPNGChunkStream* pChunk
)
{
  aErr uiErr = aErrNone;
  
  if (!pChunk)
    uiErr = aErrParam;
 
  if (uiErr == aErrNone) 
    aStreamBuffer_Create(ioRef, 1000, &pChunk->buffer, &uiErr);

  /* write the type */
  if (uiErr == aErrNone)
    aStream_Write(ioRef,
    		  pChunk->buffer,
    		  type,
    		  4,
    		  &uiErr);

  if (uiErr == aErrNone)
    pChunk->ioRef = ioRef;
  
  return uiErr;

} /* sPNGChunkStream_Init */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPNGChunkStream_Add
 */

aErr sPNGChunkStream_Add (
  aPNGChunkStream* pChunk,
  char* pData,
  unsigned long length
)
{
  aErr uiErr = aErrNone;
  
  if (!pChunk)
    uiErr = aErrParam;
  
  aAssert(pChunk->ioRef);

  if (uiErr == aErrNone) 
    aStream_Write(pChunk->ioRef, pChunk->buffer, 
    		  pData, length, &uiErr);

  return uiErr;

} /* sPNGChunkStream_Add */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * PNG CRC code
 */
  
   static void make_crc_table(void);
   static unsigned long update_crc(unsigned long crc, 
				   unsigned char *buf,
				   int len);
   static unsigned long crc(unsigned char *buf, 
			    int len);

   /* Table of CRCs of all 8-bit messages. */
   unsigned long crc_table[256];
   
   /* Flag: has the table been computed? Initially false. */
   int crc_table_computed = 0;
   
   /* Make the table for a fast CRC. */
   void make_crc_table(void)
   {
     unsigned long c;
     int n, k;
   
     for (n = 0; n < 256; n++) {
       c = (unsigned long) n;
       for (k = 0; k < 8; k++) {
         if (c & 1)
           c = 0xedb88320L ^ (c >> 1);
         else
           c = c >> 1;
       }
       crc_table[n] = c;
     }
     crc_table_computed = 1;
   }
   
   /* Update a running CRC with the bytes buf[0..len-1]--the CRC
      should be initialized to all 1's, and the transmitted value
      is the 1's complement of the final running CRC (see the
      crc() routine below)). */
   
   unsigned long update_crc(unsigned long crcVal, 
   			    unsigned char *buf,
                            int len)
   {
     unsigned long c = crcVal;
     int n;
   
     if (!crc_table_computed)
       make_crc_table();
     for (n = 0; n < len; n++) {
       c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
     }
     return c;
   }
   
   /* Return the CRC of the bytes buf[0..len-1]. */
   unsigned long crc(unsigned char *buf, int len)
   {
     return update_crc(0xffffffffL, buf, len) ^ 0xffffffffL;
   }


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPNGChunkStream_Finish
 */

aErr sPNGChunkStream_Finish ( 
  aPNGChunkStream* pChunk,
  aStreamRef output
)
{
  aErr uiErr = aErrNone;
  unsigned long len = 0;
  char* pChunkData;
  unsigned long bufferCRC = 0;
  long nl;

  if (!pChunk)
    uiErr = aErrParam;

  /* get the info for the buffer and compute checksum */
  if (uiErr == aErrNone) {
    aMemSize size;
    if (!aStreamBuffer_Get(pChunk->ioRef,
    		      pChunk->buffer,
    		      &size,
    		      &pChunkData,
    		      &uiErr)) {
      len = size - 4;
      bufferCRC = crc((unsigned char*)pChunkData, (int)size);
    }
  }

  /* write the length */
  if (uiErr == aErrNone) {
    aUtil_StoreLong((char*)&nl, (long)len);
    aStream_Write(pChunk->ioRef,
    		  output,
    		  (char*)&nl,
    		  sizeof(nl),
    		  &uiErr);
  }

  /* write the actual data */
  if (uiErr == aErrNone)
    aStream_Flush(pChunk->ioRef,
    			pChunk->buffer,
    			output,
    			&uiErr);

  if (uiErr == aErrNone)
    aStream_Destroy(pChunk->ioRef,
    		    pChunk->buffer,
    		    &uiErr);

  /* write the crc */
  if (uiErr == aErrNone) {
    aUtil_StoreLong((char*)&nl, (long)bufferCRC);
    aStream_Write(pChunk->ioRef,
    		  output,
    		  (char*)&nl,
    		  sizeof(nl),
    		  &uiErr);
  }


  if (uiErr == aErrNone)
    pChunk->ioRef = NULL;
 
  return uiErr;

} /* sPNGChunkStream_Finish */


#ifndef aPALM
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aGD_WritePNG
 */

aLIBRETURN aGD_WritePNG (
  aUILib uiRef,
  aGDRef gdRef,
  aStreamRef pngStream,
  aErr* pErr
) 
{
  aErr uiErr = aErrNone;
  aGD* pGD = (aGD*)gdRef;
  aPNGChunkStream chunk;
  aStreamRef zlibStream;
  unsigned int y;
  char* p;
  unsigned int inc;

  /* check to see if we have an offscreen */
  aGDOS* pGDOffscreen = (aGDOS*)pGD->vpData;
    char buf[4];
  
  /* write the png header signature */
  if (uiErr == aErrNone) {
    unsigned char signature[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    aStream_Write(pGD->pUI->ioRef, pngStream, 
    		  (char*)signature, sizeof(signature), &uiErr);
  }

  /* write the required IHDR chunk */
  if (uiErr == aErrNone)
    uiErr = sPNGChunkStream_Init(pGD->pUI->ioRef, "IHDR", &chunk);
  /* width (4 bytes) */
  if (uiErr == aErrNone) {
    aUtil_StoreLong(buf, (long)pGDOffscreen->nWidth);
    uiErr = sPNGChunkStream_Add(&chunk, buf, 4);
  }
  /* height (4 bytes) */
  if (uiErr == aErrNone) {
    aUtil_StoreLong(buf, (long)pGDOffscreen->nHeight);
    uiErr = sPNGChunkStream_Add(&chunk, buf, 4);
  }
  /* bit depth, color mode (2 bytes) */
  if (uiErr == aErrNone) {
    buf[0] = 8; /* 8 bits/pixel */
    buf[1] = 2; /* color used */
    uiErr = sPNGChunkStream_Add(&chunk, buf, 2);
  }
  /* compression, filter, interlace (3 bytes) */
  if (uiErr == aErrNone) {
    buf[0] = 0; /* compression type 0 */
    buf[1] = 0; /* no filter */
    buf[2] = 0; /* no interlace */
    uiErr = sPNGChunkStream_Add(&chunk, buf, 3);
  }
  if (uiErr == aErrNone)
    uiErr = sPNGChunkStream_Finish(&chunk, pngStream);
  
  
  
  /* write the required IDAT chunk */
  if (uiErr == aErrNone)
    uiErr = sPNGChunkStream_Init(pGD->pUI->ioRef, "IDAT", &chunk);

  /* it needs to be compressed */
  if (uiErr == aErrNone)
    aStream_CreateZLibFilter(pGD->pUI->ioRef,
    			     chunk.buffer,
    			     aFileModeWriteOnly,
    			     &zlibStream,
    			     &uiErr);

  if (uiErr == aErrNone) {
    p = pGDOffscreen->pPixels;
    inc = pGDOffscreen->nWidth * 3;
    buf[0] = 0; /* 0 = no line filter */
    for (y = 0; ((uiErr == aErrNone) 
  	       && (y < pGDOffscreen->nHeight)); y++, p += inc) {
      /* line filters */
      if (uiErr == aErrNone)
        aStream_Write(pGD->pUI->ioRef,
        	      zlibStream, 
        	      buf, 1L, &uiErr);
      /* the raster line */
      if (uiErr == aErrNone)
        aStream_Write(pGD->pUI->ioRef,
        	      zlibStream, 
        	      p, inc, &uiErr);
    } /* for */
  }
  
  if (uiErr == aErrNone)
    aStream_Destroy(pGD->pUI->ioRef, zlibStream, &uiErr);

  if (uiErr == aErrNone)
    uiErr = sPNGChunkStream_Finish(&chunk, pngStream);



  /* write the required IEND chunk */
  if (uiErr == aErrNone)
    uiErr = sPNGChunkStream_Init(pGD->pUI->ioRef, "IEND", &chunk);
  if (uiErr == aErrNone)
    uiErr = sPNGChunkStream_Finish(&chunk, pngStream);


  if (pErr)
    *pErr = uiErr;

  return (aLIBRETURN)(uiErr != aErrNone);

} /* aGD_WritePNG */

#endif /* ifndef aPALM */
