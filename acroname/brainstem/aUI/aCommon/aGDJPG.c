/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGDJPG.c   	                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent graphics  */
/*		offscreen drawing object.			   */
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

#ifndef aPALM

#include "aUI.h"
#include "aGDOffscreen.h"

#include <setjmp.h>

#include "jpeglib.h"
#include "jerror.h"


#define aJPG_INPUT_BUF_SIZE	4096

typedef struct aJPG_IO {
  struct jpeg_source_mgr 	pub;	/* public fields */

  aStreamRef		 	stream;
  JOCTET*		 	buffer;
  aBool				bStart;
} aJPG_IO;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static void sJPGSrc_Create(
  struct jpeg_decompress_struct* pcinfo, 
  aStreamRef inputStream);

static void sInit_Source(
  j_decompress_ptr pcinfo);

static boolean sFill_Input_Buffer(
  j_decompress_ptr pcinfo);

static void sSkip_Input_Data(
  j_decompress_ptr pcinfo, 
  long nBytes);

static void sTerm_Source(
  j_decompress_ptr pcinfo);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void sInit_Source(
  j_decompress_ptr pcinfo
)
{
  aJPG_IO* pJPGIO = (aJPG_IO*)pcinfo->src;
  pJPGIO->bStart = TRUE;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

boolean sFill_Input_Buffer(
  j_decompress_ptr pcinfo
)
{
  aJPG_IO* pJPGIO = (aJPG_IO*)pcinfo->src;
  aErr jpgErr = aErrNone;
  int nBytes = 0;
  char* p = (char*)pJPGIO->buffer;

  /* read the bytes in one at a time */
  while ((jpgErr == aErrNone) && (nBytes < aJPG_INPUT_BUF_SIZE)) {
    if (!aStream_Read(aStreamLibRef(pJPGIO->stream), pJPGIO->stream,
    		      p, 1, &jpgErr)) {
      p++;
      nBytes++;
    }
  } /* while */

  if (nBytes <= 0) {
    /* Treat empty input file as fatal error */
    if (pJPGIO->bStart)
      ERREXIT(pcinfo, JERR_INPUT_EMPTY);
    WARNMS(pcinfo, JWRN_JPEG_EOF);

    /* Insert a fake EOI marker */
    pJPGIO->buffer[0] = (JOCTET)0xFF;
    pJPGIO->buffer[1] = (JOCTET)JPEG_EOI;
    nBytes = 2;
  }

  pJPGIO->pub.next_input_byte = pJPGIO->buffer;
  pJPGIO->pub.bytes_in_buffer = nBytes;
  pJPGIO->bStart = aFalse;

  return TRUE;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void sSkip_Input_Data(
  j_decompress_ptr pcinfo, 
  long nBytes
)
{
  aJPG_IO* pJPGIO = (aJPG_IO*)pcinfo->src;

  /* Just a dumb implementation for now.  Could use fseek() except
   * it doesn't work on pipes.  Not clear that being smart is worth
   * any trouble anyway --- large skips are infrequent.
   */
  if (nBytes > 0) {
    while (nBytes > (long)pJPGIO->pub.bytes_in_buffer) {
      nBytes -= (long) pJPGIO->pub.bytes_in_buffer;
      (void)sFill_Input_Buffer(pcinfo);
      /* note we assume that fill_input_buffer will never return 
       * FALSE, so suspension need not be handled.
       */
    }
    pJPGIO->pub.next_input_byte += (size_t)nBytes;
    pJPGIO->pub.bytes_in_buffer -= (size_t)nBytes;
  }
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void sTerm_Source(
  j_decompress_ptr pcinfo
)
{
  aJPG_IO* pJPGIO = (aJPG_IO*)pcinfo->src;

  if (pJPGIO->stream) {
    aStream_Destroy(aStreamLibRef(pJPGIO->stream), 
  		    pJPGIO->stream, NULL);
    pJPGIO->stream = NULL;
  }
  aMemFree(pJPGIO->buffer);
  aMemFree(pJPGIO);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct aJPG_ERR {
  struct jpeg_error_mgr pub;		/* "public" fields */
  jmp_buf 		setjmp_buffer;	/* for return to caller */
} aJPG_ERR;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static void aJPG_Error_Exit(
  j_common_ptr pcinfo);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void aJPG_Error_Exit(
  j_common_ptr pcinfo
)
{
  aJPG_ERR* pJPGERR = (aJPG_ERR*)pcinfo->err;

  /* Always display the message. */
  /* We could postpone this until after returning, if we chose. */
/*  (*cinfo->err->output_message) (cinfo); */

  /* Return control to the setjmp point */
  longjmp(pJPGERR->setjmp_buffer, 1);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void sJPGSrc_Create(
  struct jpeg_decompress_struct* pcinfo, 
  aStreamRef inputStream
)
{
  aJPG_IO* pJPGIO = NULL;

  if (pcinfo->src == NULL) {
    pcinfo->src = 
    	(struct jpeg_source_mgr*)aMemAlloc(sizeof(aJPG_IO));
    pJPGIO = (aJPG_IO*)pcinfo->src;
    pJPGIO->buffer = 
    	(JOCTET*)aMemAlloc(sizeof(JOCTET) * aJPG_INPUT_BUF_SIZE);
  }
  
  pJPGIO = (aJPG_IO*)pcinfo->src; /* re-assign if in place */
  pJPGIO->pub.init_source = sInit_Source;
  pJPGIO->pub.fill_input_buffer = sFill_Input_Buffer;
  pJPGIO->pub.skip_input_data = sSkip_Input_Data;
  pJPGIO->pub.resync_to_restart = jpeg_resync_to_restart;
  pJPGIO->pub.term_source = sTerm_Source;
  pJPGIO->stream = inputStream;
  pJPGIO->pub.bytes_in_buffer = 0;
  pJPGIO->pub.next_input_byte = NULL;  

} /* sJPGSrc_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aLIBRETURN aGD_ReadJPG(
  aUILib uiRef,
  aStreamRef jpgStream,
  aGDRef* pGDRef,
  aErr* pErr
)
{
  aErr jpgErr = aErrNone;
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  aJPG_ERR errMgr;
  aUIPixelType width = 0;
  aUIPixelType height = 0;
  aGD* pGD = (aGD*)NULL;
  aGDOS* pGDOS = (aGDOS*)NULL;
  unsigned char* buffer;

  /* allocate the jpg error routine */
  cinfo.err = jpeg_std_error(&jerr);
  errMgr.pub.error_exit = aJPG_Error_Exit;
  if (setjmp(errMgr.setjmp_buffer)) {
    jpeg_destroy_decompress(&cinfo);
    jpgErr = aErrIO;
  }

  if (jpgErr == aErrNone) {
    /* Now we can initialize the JPEG decompression object. */
    jpeg_create_decompress(&cinfo);
  
    /* Specify the data source */
    sJPGSrc_Create(&cinfo, jpgStream);

    jpeg_read_header(&cinfo, TRUE);
  
    width = cinfo.image_width;
    height = cinfo.image_height;

    /* build the receiving offscreen data block */
    aGDInternal_CreateOffscreen((aUI*)uiRef,
  			        width, height,
  			        pGDRef, &jpgErr);  
    if (jpgErr == aErrNone) {
      pGD = (aGD*)*pGDRef;
      pGDOS = (aGDOS*)pGD->vpData;
      buffer = (unsigned char*)pGDOS->pPixels;

      /* set to RGB output */
      cinfo.out_color_space = JCS_RGB;

      jpeg_start_decompress(&cinfo);

      while ((aUIPixelType)cinfo.output_scanline < height) {
        jpeg_read_scanlines(&cinfo, &buffer, (unsigned int)1);
        buffer += width * 3;
      }

      jpeg_finish_decompress(&cinfo);

      jpeg_destroy_decompress(&cinfo);
    }
  }

  if (pErr != NULL)
    *pErr = jpgErr;

  return (aLIBRETURN)(jpgErr != aErrNone);
}
#endif /* aPALM */
