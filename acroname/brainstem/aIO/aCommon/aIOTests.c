/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aIOTests.c                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Testing module for the aIO shared library.         */
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

#include "aIO.h"
#include "aUtil.h"
#include "aIOTests.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static void sTest_Print(const char* text,
			aStreamRef out);

static aBool sTest_aFile_Open(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aFile_Close(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aFile_Read(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aFile_Write(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aFile_Seek(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aFile_GetSize(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aFile_Delete(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aFile_Use(aIOLib ioLibRef, aStreamRef out);

static aBool sTest_aIO_GetMSTicks(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aIO_MSSleep(aIOLib ioLibRef, aStreamRef out);

static aBool sTest_aTokenizer_Create(aIOLib ioLibRef, aStreamRef out);
static aBool sTest_aTokenizer_Next(aIOLib ioLibRef, aStreamRef out);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_Write
 */

void sTest_Print(const char* text,
		 aStreamRef out)
{
  aStream_Write(aStreamLibRef(out), out, text, 
  		aStringLen(text), NULL);

} /* sTest_Print */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Open
 */

aBool sTest_aFile_Open(aIOLib ioLibRef, 
		       aStreamRef out)
{
  aErr err;
  char msg[100];
  aFileRef fileRef;
  char filename[aFILE_NAMEMAXCHARS+2];
  char testwritefilename[aFILE_NAMEMAXCHARS+2];
  aFileRef testwritefileRef;
  int i;
  unsigned long uSize;

  aStream_WriteLine(ioLibRef, out, "  aFile_Open", NULL);

  aStringCopy(msg, "   NULL Lib Ref (read)..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(NULL, "test", aFileModeReadOnly, aFileAreaUser,
  	     &fileRef, &err);
  if (err != aErrParam)
    return aTrue;
  if (aFile_Open(NULL, "test", aFileModeReadOnly, aFileAreaUser,
  		 &fileRef, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   NULL Lib Ref (write)..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(NULL, "test", aFileModeWriteOnly, aFileAreaUser,
  	     &fileRef, &err);
  if (err != aErrParam)
    return aTrue;
  if (aFile_Open(NULL, "test", aFileModeWriteOnly, aFileAreaUser,
  		 &fileRef, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening a file with a NULL file ref both with and 
   * without an error pointer in read mode
   */
  aStringCopy(msg, "   NULL File Ref (read)");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Open(ioLibRef, "test", aFileModeReadOnly, 
  		  aFileAreaUser, NULL, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, "test", aFileModeReadOnly, 
 		 aFileAreaUser, NULL, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening a file with a NULL file ref both with and 
   * without an error pointer in write mode
   */
  aStringCopy(msg, "   NULL File Ref (write)");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Open(ioLibRef, "test", aFileModeWriteOnly, 
  		  aFileAreaUser, NULL, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, "test", aFileModeWriteOnly, 
  		 aFileAreaUser, NULL, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening a file with a NULL filename in read mode
   */
  aStringCopy(msg, "   NULL File Name (read)");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Open(ioLibRef, NULL, aFileModeReadOnly, 
  		  aFileAreaUser, &fileRef, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, "test", aFileModeWriteOnly, 
  		 aFileAreaUser, NULL, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   NULL File Name (write)");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Open(ioLibRef, NULL, aFileModeWriteOnly, 
  		  aFileAreaUser, &fileRef, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, "test", aFileModeWriteOnly, 
  		 aFileAreaUser, NULL, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening a file that doesn't exist in read mode
   */
  aStringCopy(msg, "   Non-existent file");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Open(ioLibRef, "non-existent", aFileModeReadOnly, 
  		  aFileAreaUser, &fileRef, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrNotFound)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, "non_existent", aFileModeReadOnly, 
  		 aFileAreaUser, &fileRef, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening a file with a name that is too long in read mode
   */
  for (i = 0; i < aFILE_NAMEMAXCHARS + 1; i++)
    filename[i] = 'a';
  filename[i] = '\0';
  aStringCopy(msg, "   Filename too long (read)");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Open(ioLibRef, filename, aFileModeReadOnly, 
  		  aFileAreaUser, &fileRef, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrFileNameLength)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, filename, aFileModeReadOnly, 
  		 aFileAreaUser, &fileRef, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening a file with a name that is too long in write mode
   */
  aStringCopy(msg, "   Filename too long (write)");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Open(ioLibRef, filename, aFileModeWriteOnly, 
  		  aFileAreaUser, &fileRef, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrFileNameLength)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, filename, aFileModeWriteOnly, 
  		 aFileAreaUser, &fileRef, NULL) == 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening an actual file in write mode with an error pointer
   */
  aStringCopy(testwritefilename, "testwritefile");

  aStringCopy(msg, "   Open \"testwritefile\" writing");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  		 aFileAreaUser, &testwritefileRef, &err))
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", closing");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Close(ioLibRef, testwritefileRef, NULL))
    return aTrue;
  aStringCopy(msg, ", deleting...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Delete(ioLibRef, testwritefilename, aFileAreaUser, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* open a file, write to it, close and reopen for writing, close
   * and check size, should be zero length.
   */
  aStringCopy(msg, "   Reopen for writing zero length");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  		 aFileAreaUser, &testwritefileRef, &err))
    return aTrue;
  aStringCopy(msg, ", writing");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Write(ioLibRef, testwritefileRef, "test", 5, NULL, NULL))
    return aTrue;
  aStringCopy(msg, ", closing");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Close(ioLibRef, testwritefileRef, NULL))
    return aTrue;
  aStringCopy(msg, ", reopen");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  		 aFileAreaUser, &testwritefileRef, &err))
    return aTrue;
  aStringCopy(msg, ", sizing...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_GetSize(ioLibRef, testwritefileRef, &uSize, NULL))
    return aTrue;
  if (uSize != 0)
    return aTrue;
  aStringCopy(msg, ", closing");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Close(ioLibRef, testwritefileRef, NULL))
    return aTrue;
  aStringCopy(msg, ", deleting...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Delete(ioLibRef, testwritefilename, aFileAreaUser, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);




  /* open a file for writing and then try to open another to the 
   * same file, should return aErrBusy.
   */
  aStringCopy(msg, "   Reopen file for writing");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  		 aFileAreaUser, &testwritefileRef, &err))
    return aTrue;

#ifndef aUNIX
  /* Unix doesn't handle the notion of exclusive access to files 
   * in a way we can generalize so we don't check for busy status
   * on already open files. */
  {
    aFileRef duplicateFileRef;
    aStringCopy(msg, ", opening again");
    aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
    if (!aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
		    aFileAreaUser, &duplicateFileRef, &err))
      return aTrue;
    aStringCopy(msg, ", error code check");
    aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
    if (err != aErrBusy)
      return aTrue;
  }
#endif

  aStringCopy(msg, ", closing");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Close(ioLibRef, testwritefileRef, NULL))
    return aTrue;
  aStringCopy(msg, ", deleting...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Delete(ioLibRef, testwritefilename, aFileAreaUser, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  return aFalse;

} /* sTest_aFile_Open */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Close
 */

aBool sTest_aFile_Close(aIOLib ioLibRef, 
		        aStreamRef out)
{
  aErr err;
  char msg[100];
  aFileRef fileRef;
  aFileRef badFileRef;
  char testclosefilename[aFILE_NAMEMAXCHARS+2];  /* UNL I added this to help w/ consistency */

  aStream_WriteLine(ioLibRef, out, "  aFile_Close", NULL);

  aStringCopy(testclosefilename, "test_fc");  

  badFileRef=(void*)42; /* junk pointer */
  
  /* try closing a file with a NULL lib ref
   */

  aStringCopy(msg, "   NULL Lib Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, testclosefilename , aFileModeWriteOnly, 
  		 aFileAreaUser, &fileRef, &err))
    return aTrue;  
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Close(NULL, fileRef, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Close(NULL,fileRef,NULL)) /* should fail */
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err); /* good close */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try closing a file with a NULL file ref
   */

  aStringCopy(msg, "   NULL File Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);


  aFile_Open(ioLibRef, testclosefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Close(ioLibRef, NULL, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Close(ioLibRef, NULL, NULL)) /* should fail */
    return aTrue;
  aFile_Close(ioLibRef, fileRef, &err); /* good close */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try closing a file with a BAD file ref
   * THIS CAUSED A BIG FAT CRASH
   */
/*
  aStringCopy(msg, "   BAD File Ref...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, "test_fc", aFileModeWriteOnly, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Close(ioLibRef,badFileRef,&err);
  if (err != aErrParam)
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
*/

  /* try closing a file multiple times
   * (DEBUG was catching a extra de-allocation)
   */
  aStringCopy(msg, "   Multiple Close");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testclosefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Close(ioLibRef, fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error on 2nd");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Close(ioLibRef, fileRef, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Close(ioLibRef, fileRef, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try opening and closing existing file
   */
  aStringCopy(msg, "   Existing Close...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testclosefilename, aFileModeReadOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Close(ioLibRef, fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* get rid of our working file 
   */
  aFile_Delete(ioLibRef, testclosefilename, aFileAreaUser, &err);
  if (err != aErrNone)
    return aTrue;

  return aFalse;

} /* sTest_aFile_Close */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Read
 */

aBool sTest_aFile_Read(aIOLib ioLibRef, 
		       aStreamRef out)
{
  aErr err;
  char msg[100];
  char buff[100];
  char guff[100];
  aFileRef fileRef;
  unsigned long uwCt;
  int nCmpRes;
  char testreadfilename[aFILE_NAMEMAXCHARS+2];  /* UNL I added this to help w/ consistency */

  aStringCopy(testreadfilename,"test_rd");  

  aStream_WriteLine(ioLibRef, out, "  aFile_Read", NULL);

  /* create a generic test file */
  /* bad param tests come first so write-only won't matter */
  
  aFile_Open(ioLibRef, testreadfilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(buff, "0123456789\n");
  aFile_Write(ioLibRef, fileRef, buff, aStringLen(buff), &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != aStringLen(buff))
    return aTrue;

  /* try reading with NULL lib ref
   */

  aStringCopy(msg, "   NULL Lib Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Read(NULL,fileRef,guff,10,&uwCt,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Read(NULL,fileRef,guff,10,&uwCt,NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* try reading with NULL file ref
   */

  aStringCopy(msg, "   NULL File Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Read(ioLibRef, NULL, guff, 10, &uwCt, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Read(ioLibRef, NULL, guff, 10, &uwCt, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* try reading with NULL read buffer
   */

  aStringCopy(msg, "   NULL Read Buffer");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Read(ioLibRef, fileRef, NULL, 10, &uwCt, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Read(ioLibRef, fileRef, NULL, 10, &uwCt, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* try reading from write only file
   */

  aStringCopy(msg, "   Read From Write Only");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Read(ioLibRef, fileRef, guff, 10, &uwCt, &err);
  if (err != aErrMode)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Read(ioLibRef, fileRef, guff, 10, &uwCt, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* close working file */
  /* and re-open file as read only */

  aFile_Close(ioLibRef, fileRef, &err);
  if (err != aErrNone)
    return aTrue;

  aFile_Open(ioLibRef, testreadfilename, aFileModeReadOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;


  /* try reading properly
   */

  aStringCopy(msg, "   Read From Read Only");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(guff, "aaaaaaaaaa");
  aFile_Read(ioLibRef, fileRef, guff, 10, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 10)
    return aTrue;
  nCmpRes = aStringCompare(guff, "0123456789");
  if (nCmpRes) /* 0 means equal */
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Seek(ioLibRef, fileRef, 0, aTrue, &err);
  if (err != aErrNone)
    return aTrue;
  if (aFile_Read(ioLibRef, fileRef, guff, 10, &uwCt, NULL)) /* should succeed */
    return aTrue;
  if (uwCt != 10)
    return aTrue;
  nCmpRes = aStringCompare(guff, "0123456789");
  if (nCmpRes) /* 0 means equal */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aFile_Close(ioLibRef, fileRef, &err);   /* close working file */
  if (err != aErrNone)
    return aTrue;

  /* try reading from closed file
   */

  aStringCopy(msg, "   Read From Closed File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(guff, "aaaaaaaaaa");
  aFile_Read(ioLibRef, fileRef, guff, 10, &uwCt, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Read(ioLibRef, fileRef, guff, 10, &uwCt, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* erase working file */
  aFile_Delete(ioLibRef, testreadfilename, aFileAreaUser, &err);
  if (err != aErrNone)
    return aTrue;

  return aFalse;
  
} /* sTest_aFile_Read */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Write
 */

aBool sTest_aFile_Write(aIOLib ioLibRef, 
		        aStreamRef out)
{

  aErr err;
  char msg[100];
  char buff[100];
  aFileRef fileRef;
  unsigned long uwCt;

  char testwritefilename[aFILE_NAMEMAXCHARS+2];  /* UNL I added this to help w/ consistency */

  aStringCopy(testwritefilename,"test_wr");  
  
  aStream_WriteLine(ioLibRef, out, "  aFile_Write", NULL);

  /* try writing with NULL lib ref
   */

  aStringCopy(msg, "   NULL Lib Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(buff, "0123456789\n");
  aFile_Write(NULL,fileRef,buff,aStringLen(buff),&uwCt,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Write(NULL,fileRef,buff,aStringLen(buff),&uwCt,NULL)) /* should fail */
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);   /* close working file */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try writing with NULL file ref
   */

  aStringCopy(msg, "   NULL File Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(buff, "0123456789\n");
  aFile_Write(ioLibRef,NULL,buff,aStringLen(buff),&uwCt,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Write(ioLibRef,NULL,buff,aStringLen(buff),&uwCt,NULL)) /* should fail */
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);   /* close working file */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try writing with NULL buffer pointer
   */

  aStringCopy(msg, "   NULL Buffer Ptr");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Write(ioLibRef,fileRef,NULL,0,&uwCt,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Write(ioLibRef,fileRef,NULL,0,&uwCt,NULL)) /* should fail */
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);   /* close working file */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try writing to a write-only file
   */

  aStringCopy(msg, "   Write to Write Only");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(buff, "0123456789\n");
  aFile_Write(ioLibRef,fileRef,buff,aStringLen(buff),&uwCt,&err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != aStringLen(buff))
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Write(ioLibRef,fileRef,buff,aStringLen(buff),&uwCt,NULL)) /* should succeed */
    return aTrue;
  if (uwCt != aStringLen(buff))
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);   /* close working file */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try writing with an empty buffer
   */

  aStringCopy(msg, "   Write Empty Buffer");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(buff, ""); /* empty */
  aFile_Write(ioLibRef,fileRef,buff,aStringLen(buff),&uwCt,&err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != aStringLen(buff))
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Write(ioLibRef,fileRef,buff,aStringLen(buff),&uwCt,NULL)) /* should succeed */
    return aTrue;
  if (uwCt != aStringLen(buff))
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);   /* close working file */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try writing with occupied buffer but 0 char
   */

  aStringCopy(msg, "   Write Zero Bytes");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testwritefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(buff, "0123456789\n");
  aFile_Write(ioLibRef,fileRef,buff,0,&uwCt,&err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 0)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Write(ioLibRef,fileRef,buff,0,&uwCt,NULL)) /* should succeed */
    return aTrue;
  if (uwCt != 0)
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);   /* close working file */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* try writing to closed file
   */

  aStringCopy(msg, "   Write to Closed File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(buff, "0123456789\n");
  aFile_Write(ioLibRef,fileRef,buff,0,&uwCt,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Write(ioLibRef,fileRef,buff,0,&uwCt,NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  
  /* try writing to a read-only file
   */

  aStringCopy(msg, "   Write to Read Only");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testwritefilename, aFileModeReadOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(buff, "0123456789\n");
  aFile_Write(ioLibRef,fileRef,buff,aStringLen(buff),&uwCt,&err);
  if (err != aErrMode)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Write(ioLibRef,fileRef,buff,aStringLen(buff),&uwCt,NULL)) /* should fail */
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);   /* close working file */
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* NO EASY WAY TO TEST FOR aErrWrite */

  /* erase working file */
  aFile_Delete(ioLibRef, testwritefilename, aFileAreaUser, &err);
  if (err != aErrNone)
    return aTrue;
  
  return aFalse;
  
} /* sTest_aFile_Write */






/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Seek
 */

aBool sTest_aFile_Seek(aIOLib ioLibRef, 
		       aStreamRef out)
{

  aErr err;
  char msg[100];
  char buff[100];
  char guff[100];
  aFileRef fileRef;
  unsigned long uwCt;
  char testseekfilename[aFILE_NAMEMAXCHARS+2];  /* UNL I added this to help w/ consistency */

  aStringCopy(testseekfilename, "test_sk");  
  
  aStream_WriteLine(ioLibRef, out, "  aFile_Seek", NULL);

  /* create a generic test file */
  /* bad param tests come first so write-only won't matter */
  
  aFile_Open(ioLibRef, testseekfilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(buff, "0123456789\n");
  aFile_Write(ioLibRef, fileRef, buff, aStringLen(buff), &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != aStringLen(buff))
    return aTrue;

  /* try seeking with NULL lib ref
   */

  aStringCopy(msg, "   NULL Lib Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Seek(NULL,fileRef,4,aTrue,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Seek(NULL, fileRef, 1, aFalse, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try seeking with NULL file ref */

  aStringCopy(msg, "   NULL File Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Seek(ioLibRef,NULL,4,aTrue,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Seek(ioLibRef, NULL, 1, aFalse, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* close working file (11 bytes long) */
  /* and re-open file as read only */
  
  aFile_Close(ioLibRef,fileRef,&err);
  if (err != aErrNone)
    return aTrue;
  aFile_Open(ioLibRef, testseekfilename, aFileModeReadOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;

  /* try seeking normally from beginning and from current pos
   */
  aStringCopy(msg, "   Seek Normally F&B");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(guff, "aaaaaaaaaa");
  /* move to byte 4 */
  aFile_Seek(ioLibRef, fileRef, 4, aTrue, &err);
  if (err != aErrNone)
    return aTrue;
  /* advances 1 byte */
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '4')
    return aTrue;
  /* advances 1 more byte */
  aFile_Seek(ioLibRef, fileRef, 1, aFalse, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '6')
    return aTrue;
  /* back up two bytes */
  aFile_Seek(ioLibRef, fileRef, -2, aFalse, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '5')
    return aTrue;

  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(guff,"aaaaaaaaaa");
  if (aFile_Seek(ioLibRef, fileRef, 4, aTrue, &err)) /* should succeed */
    return aTrue;
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err); /* advances 1 byte */
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '4')
    return aTrue;
  if (aFile_Seek(ioLibRef, fileRef, 1, aFalse, &err)) /* should succeed */
    return aTrue; /* seek advances 1 more byte */
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '6')
    return aTrue;
  if (aFile_Seek(ioLibRef, fileRef, -2, aFalse, &err)) /* should succeed */
    return aTrue; /* seek backs up two bytes */
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '5')
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* now try seeking forward past EOF
   */

  aStringCopy(msg, "   Seek Forward Past EOF");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  aFile_Seek(ioLibRef, fileRef, 5, aTrue, &err); /* start at offset 5 */
  if (err != aErrNone)
    return aTrue;
  aStringCopy(guff,"aaaaaaaaaa");

  aFile_Seek(ioLibRef, fileRef, 20, aTrue, &err);
  if (err != aErrEOF)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  if (!aFile_Seek(ioLibRef, fileRef, 20, aTrue, NULL)) /* should fail */
    return aTrue;
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '5') /* must end at offset 5 */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* now try seeking backward past EOF
   */

  aStringCopy(msg, "   Seek Backward Past EOF");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(guff,"aaaaaaaaaa");
  aFile_Seek(ioLibRef, fileRef, 5, aTrue, &err); /* start at offset 5 */
  if (err != aErrNone)
    return aTrue;
  aFile_Seek(ioLibRef, fileRef, -20, aTrue, &err);
  if (err != aErrEOF)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Seek(ioLibRef, fileRef, -20, aTrue, NULL)) /* should fail */
    return aTrue;
  aFile_Read(ioLibRef, fileRef, guff, 1, &uwCt, &err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != 1)
    return aTrue;
  if (guff[0] != '5') /* must end at offset 5 */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  aFile_Close(ioLibRef, fileRef, &err);   /* close working file */
  if (err != aErrNone)
    return aTrue;


  /* try seeking in a closed file
   */

  aStringCopy(msg, "   Seek in Closed File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(guff,"aaaaaaaaaa");
  aFile_Seek(ioLibRef, fileRef, 5, aTrue, &err); /* start at offset 5 */
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Seek(ioLibRef, fileRef, 5, aTrue, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* erase working file */
  aFile_Delete(ioLibRef, testseekfilename, aFileAreaUser, &err);
  if (err != aErrNone)
    return aTrue;

  return aFalse;

} /* sTest_aFile_Seek */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Seek
 */

aBool sTest_aFile_GetSize(aIOLib ioLibRef, 
		          aStreamRef out)
{

  aErr err;
  char msg[100];
  char buff[100];
  aFileRef fileRef;
  unsigned long uwCt;
  unsigned long uSize;
  char testgetsizefilename[aFILE_NAMEMAXCHARS+2];  /* UNL I added this to help w/ consistency */

  aStringCopy(testgetsizefilename,"test_gs.txt");  
 
  aStream_WriteLine(ioLibRef, out, "  aFile_GetSize", NULL);

  /* create a generic test file */
  /* bad param tests come first so write-only won't matter */
  /* (don't use slash-n so file size is consistent across platforms) */
  
  aFile_Open(ioLibRef, testgetsizefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aStringCopy(buff, "0123456789");
  aFile_Write(ioLibRef,fileRef,buff,aStringLen(buff),&uwCt,&err);
  if (err != aErrNone)
    return aTrue;
  if (uwCt != aStringLen(buff))
    return aTrue;

  /* get size with NULL lib ref
   */

  aStringCopy(msg, "   NULL Lib Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_GetSize(NULL,fileRef,&uSize,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_GetSize(NULL,fileRef,&uSize,NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* get size with NULL file ref
   */

  aStringCopy(msg, "   NULL File Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_GetSize(ioLibRef,NULL,&uSize,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_GetSize(ioLibRef, NULL, &uSize, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* get size for write-only file
   */

  aStringCopy(msg, "   Size of Write Only File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  uSize=0;
  aFile_GetSize(ioLibRef,fileRef,&uSize,&err);
  if (err != aErrNone)
    return aTrue;
  if (uSize != 10)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  uSize=0;
  if (aFile_GetSize(ioLibRef,fileRef,&uSize,NULL)) /* should succeed */
    return aTrue;
  if (uSize != 10)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* close working file */
  /* and re-open file as read only */
  
  aFile_Close(ioLibRef,fileRef,&err);
  if (err != aErrNone)
    return aTrue;
  aFile_Open(ioLibRef, testgetsizefilename, aFileModeReadOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;

  /* get size for read-only file
   */

  aStringCopy(msg, "   Size of Read Only File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  uSize=0;
  aFile_GetSize(ioLibRef,fileRef,&uSize,&err);
  if (err != aErrNone)
    return aTrue;
  if (uSize != 10)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  uSize=0;
  if (aFile_GetSize(ioLibRef,fileRef,&uSize,NULL)) /* should succeed */
    return aTrue;
  if (uSize != 10)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aFile_Close(ioLibRef,fileRef,&err);
  if (err != aErrNone)
    return aTrue;

  /* close working file */    

  /* get size of closed file
   */

  aStringCopy(msg, "   Size of Closed File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  uSize=0;
  aFile_GetSize(ioLibRef,fileRef,&uSize,&err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  uSize=0;
  if (!aFile_GetSize(ioLibRef,fileRef,&uSize,NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* destroy working file */
  aFile_Delete(ioLibRef, testgetsizefilename, aFileAreaUser, &err);
  if (err != aErrNone)
    return aTrue;

  return aFalse;

} /* sTest_aFile_GetSize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Delete
 */

aBool sTest_aFile_Delete(aIOLib ioLibRef, 
		         aStreamRef out)
{

  aErr err;
  char msg[100];
  aFileRef fileRef;
  char testdeletefilename[aFILE_NAMEMAXCHARS+2];  /* UNL I added this to help w/ consistency */

  aStringCopy(testdeletefilename,"test_del");  
 
  aStream_WriteLine(ioLibRef, out, "  aFile_Delete", NULL);

  /* try deleting with NULL lib ref */

  aStringCopy(msg, "   NULL Lib Ref");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef,testdeletefilename,aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  aFile_Close(ioLibRef,fileRef,&err);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Delete(NULL, testdeletefilename, aFileAreaUser, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Delete(NULL, testdeletefilename, aFileAreaUser, NULL)) /* should fail */
    return aTrue;
  aFile_Delete(ioLibRef, testdeletefilename, aFileAreaUser, &err);
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try deleting with NULL file name */

  aStringCopy(msg, "   NULL File Name");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Delete(ioLibRef, NULL, aFileAreaUser, &err);
  if (err != aErrParam)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Delete(ioLibRef, NULL, aFileAreaUser, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try deleting a non-existent file */

  aStringCopy(msg, "   Delete Non-Existent File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Delete(ioLibRef, "non_existent_file", aFileAreaUser, &err);
  if (err != aErrNotFound)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Delete(ioLibRef, "non_existent_file", aFileAreaUser, NULL)) /* should fail */
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* try to delete open file */

#if !defined(aUNIX)
  aStringCopy(msg, "   Delete Open File");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aStringCopy(msg, ", error val");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testdeletefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Delete(ioLibRef, testdeletefilename, aFileAreaUser, &err);
  if (err != aErrIO)
    return aTrue;
  aStringCopy(msg, ", null error...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aFile_Delete(ioLibRef, "test_del", aFileAreaUser, NULL)) /* should fail */
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);
  if (err != aErrNone)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
#endif /* !aUNIX */

  /* try to open deleted file (verify that it is deleted) */

  aStringCopy(msg, "   Open Deleted File...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aFile_Open(ioLibRef, testdeletefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Close(ioLibRef,fileRef,&err);
  if (err != aErrNone)
    return aTrue;
  aFile_Delete(ioLibRef, testdeletefilename, aFileAreaUser, &err);
  if (err != aErrNone)
    return aTrue;
  aFile_Open(ioLibRef, testdeletefilename, aFileModeReadOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNotFound)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  return aFalse;
    
} /* sTest_aFile_Delete */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aFile_Use
 */

aBool sTest_aFile_Use(aIOLib ioLibRef, 
		      aStreamRef out)
{

  aErr err;
  char msg[100];
  aFileRef fileRef;
  unsigned char data[6];
  unsigned long bytes;
  char testusefilename[aFILE_NAMEMAXCHARS+2];

  aStringCopy(testusefilename,"test_use");  
 
  aStream_WriteLine(ioLibRef, out, "  aFile Usage", NULL);

  /* try opening writing and closing some known bytes */

  aStringCopy(msg, "   Usage, open, ");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  aFile_Open(ioLibRef, testusefilename, aFileModeWriteOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;

  aStringCopy(msg, "write, ");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  data[0] = 43;
  data[1] = 32;
  data[2] = 45;
  data[3] = 78;
  data[4] = 43;
  data[5] = 56;
  
  aFile_Write(ioLibRef, fileRef, (char*)data, 6, &bytes, &err);
  if (err != aErrNone)
    return aTrue;
  if (bytes != 6)
    return aTrue;

  aStringCopy(msg, "close...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  aFile_Close(ioLibRef, fileRef, &err);
  if (err != aErrNone)
    return aTrue;

  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* now re-open, check size, read bytes, and close */
  fileRef = NULL;
  aStringCopy(msg, "   Usage, open, ");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  aFile_Open(ioLibRef, testusefilename, aFileModeReadOnly, 
  	     aFileAreaUser, &fileRef, &err);
  if (err != aErrNone)
    return aTrue;

  aStringCopy(msg, "size, ");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  
  aFile_GetSize(ioLibRef, fileRef, &bytes, &err);
  if (err != aErrNone)
    return aTrue;
  if (bytes != 6)
    return aTrue;

  aStringCopy(msg, "read, ");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  aFile_Read(ioLibRef, fileRef, (char*)data, 6, &bytes, &err);
  if (err != aErrNone)
    return aTrue;
  if (bytes != 6)
    return aTrue;
  if ((data[0] != 43)
      || (data[1] != 32)
      || (data[2] != 45)
      || (data[3] != 78)
      || (data[4] != 43)
      || (data[5] != 56))
    return aTrue;

  aStringCopy(msg, "close...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  aFile_Close(ioLibRef, fileRef, &err);
  if (err != aErrNone)
    return aTrue;

  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   Usage, delete...");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  aFile_Delete(ioLibRef, testusefilename, aFileAreaUser, &err); 
  if (err != aErrNone)
    return aTrue;

  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  return aFalse;
    
} /* sTest_aFile_Use */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aIO_GetMSTicks
 */

aBool sTest_aIO_GetMSTicks(aIOLib ioLibRef, aStreamRef out)
{
  aErr ioErr;
  unsigned long t1, t2;
  char msg[100];

  if (aIO_GetMSTicks(ioLibRef, &t1, &ioErr))
    return aTrue;
  if (ioErr != aErrNone)
    return aTrue;
  if (aIO_GetMSTicks(ioLibRef, &t1, NULL))
    return aTrue;

  aStringCopy(msg, "   GetMSTicks, NULL lib..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aIO_GetMSTicks(NULL, NULL, NULL))
    return aTrue;
  aIO_GetMSTicks(NULL, NULL, &ioErr); 
  if (ioErr != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   GetMSTicks, NULL time value..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aIO_GetMSTicks(ioLibRef, NULL, &ioErr))
    return aTrue;
  aIO_GetMSTicks(ioLibRef, NULL, &ioErr); 
  if (ioErr != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* waste some time to make sure time moved forward*/
  if (aIO_MSSleep(ioLibRef, 10, NULL))
    return aTrue;
  
  aStringCopy(msg, "   GetMSTicks, time moves on..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aIO_GetMSTicks(ioLibRef, &t2, &ioErr))
    return aTrue;
  if (t1 >= t2) {
    aStringCopy(msg, " no time passed");
    aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
    return aTrue;
  }
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  return aFalse;

} /* sTest_aIO_GetMSTicks */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aIO_MSSleep
 */

aBool sTest_aIO_MSSleep(aIOLib ioLibRef, aStreamRef out)
{
  aErr ioErr = aErrNone;
  char msg[100];
  unsigned long start, end;

  aStringCopy(msg, "   MSSleep, basic..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aIO_GetMSTicks(ioLibRef, &start, &ioErr))
    return aTrue;
  if (aIO_MSSleep(ioLibRef, 1000, &ioErr))
    return aTrue;
  if (aIO_GetMSTicks(ioLibRef, &end, &ioErr))
    return aTrue;
  if (end - start < 950)
    return aTrue;
  if (end - start > 1050)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   MSSleep, NULL error..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aIO_GetMSTicks(ioLibRef, &start, &ioErr))
    return aTrue;  
  if (aIO_MSSleep(ioLibRef, 1000, NULL))
    return aTrue;
  if (aIO_GetMSTicks(ioLibRef, &end, &ioErr))
    return aTrue;
  if (end - start < 950)
    return aTrue;
  if (end - start > 1050)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   MSSleep, no time..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aIO_GetMSTicks(ioLibRef, &start, &ioErr))
    return aTrue;  
  if (aIO_MSSleep(ioLibRef, 0, NULL))
    return aTrue;
  if (aIO_GetMSTicks(ioLibRef, &end, &ioErr))
    return aTrue;
  if (end - start > 1)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  return aFalse;

} /* sTest_aIO_MSSleep */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aTokenizer_Create
 */

aBool sTest_aTokenizer_Create(aIOLib ioLibRef, aStreamRef out)
{
  aErr err;
  char msg[100];
  aStreamRef tokenStream = 0;
  aTokenizerRef tokenizerRef;
  aFileRef file;

  aStream_WriteLine(ioLibRef, out, "  aTokenizer_Create", NULL);

  aStringCopy(msg, "   NULL Lib Ref..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aTokenizer_Create(NULL, tokenStream, "test", aFileAreaUser,
		    NULL, NULL, &tokenizerRef, &err);
  if (err != aErrParam)
    return aTrue;
  if (!aTokenizer_Create(NULL, &tokenStream, "test", aFileAreaUser,
  	                 NULL, NULL, tokenizerRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   NULL Tokenizer Ref..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aTokenizer_Create(ioLibRef, tokenStream, "test", aFileAreaUser,
  	     NULL, NULL, NULL, &err);
  if (err != aErrParam)
    return aTrue;
  if (!aTokenizer_Create(ioLibRef, tokenStream, "test", aFileAreaUser,
  	                 NULL, NULL, NULL, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   NULL Token Stream..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aTokenizer_Create(ioLibRef, NULL, "test", aFileAreaUser,
  	            NULL, NULL, &tokenizerRef, &err);
  if (err != aErrParam)
    return aTrue;
  if (!aTokenizer_Create(ioLibRef, NULL, "test", aFileAreaUser,
  	                 NULL, NULL, &tokenizerRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* create a file for testing and create a working input stream 
   * from the test file */
  
  aStringCopy(msg, "   Create temp file..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Open(ioLibRef, "tokenfile.txt", aFileModeWriteOnly, 
  	         aFileAreaTest, &file, NULL))
    return aTrue;
  if (aFile_Write(ioLibRef, file, "\n", 1, NULL, NULL))
    return aTrue;
  if (aFile_Write(ioLibRef, file, "line 2\r\n", 8, NULL, NULL))
    return aTrue;
  if (aFile_Write(ioLibRef, file, "line 3\r", 7, NULL, NULL))
    return aTrue;
  if (aFile_Write(ioLibRef, file, "line 4\n", 7, NULL, NULL))
    return aTrue;
  if (aFile_Write(ioLibRef, file, "line 5 <test>\n", 14, NULL, NULL))
    return aTrue;

  if (aFile_Close(ioLibRef, file, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   Open temp input stream..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStream_CreateFileInput(ioLibRef, "tokenfile.txt", 
  	         aFileAreaTest, &tokenStream, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   Create and destroy..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  aTokenizer_Create(ioLibRef, tokenStream, "tokenfile.txt", 
  		    aFileAreaTest, NULL, NULL, &tokenizerRef, &err);
  if (err != aErrNone)
    return aTrue;
  aTokenizer_Destroy(ioLibRef, tokenizerRef, &err);
  if (err != aErrNone)
    return aTrue;
  if (aStream_CreateFileInput(ioLibRef, "tokenfile.txt", 
  	         aFileAreaTest, &tokenStream, NULL))
    return aTrue;
  if (aTokenizer_Create(ioLibRef, tokenStream, "test", 
  			 aFileAreaUser, NULL, NULL, &tokenizerRef, NULL))
    return aTrue;
  if (aTokenizer_Destroy(ioLibRef, tokenizerRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

#if 0
  /* clean up test input file */
  aStringCopy(msg, "   Delete temp input file..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aFile_Delete(ioLibRef, "tokenfile.txt", aFileAreaTest, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
#endif

  return(aFalse);

} /* sTest_aTokenizer_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTest_aTokenizer_Next
 */

aBool sTest_aTokenizer_Next(aIOLib ioLibRef, aStreamRef out)
{
  aErr err;
  aStreamRef tokenStream = 0;
  aTokenizerRef tokenizerRef;
  aToken* pToken;

  aStream_WriteLine(ioLibRef, out, "  aTokenizer_Next", NULL);

  /* build a buffer to feed the tokenizer with */
  sTest_Print("   build test token buffer..", out);
  if (aStreamBuffer_Create(ioLibRef, 100, &tokenStream, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* build the tokenizer for the tests */
  sTest_Print("   build test tokenizer..", out);
  if (aTokenizer_Create(ioLibRef, tokenStream, "test", aFileAreaUser,
  	                 NULL, NULL, &tokenizerRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* do some integer checks */
  sTest_Print("   testing int tokens..", out);
  if (aStream_WriteLine(ioLibRef, tokenStream, "0 1 2", NULL))
    return aTrue;

  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkInt)
    return aTrue;
  if (pToken->v.integer != 0)
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;

  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkInt)
    return aTrue;
  if (pToken->v.integer != 1)
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;

  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkInt)
    return aTrue;
  if (pToken->v.integer != 2)
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* do some float checks */
  sTest_Print("   testing float tokens..", out);
  if (aStream_WriteLine(ioLibRef, tokenStream, "0.1 .1 0.2", NULL))
    return aTrue;
  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkFloat)
    return aTrue;
  if (pToken->v.floatVal != 0.1f)
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkFloat)
    return aTrue;
  if (pToken->v.floatVal != 0.1f)
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkFloat)
    return aTrue;
  if (pToken->v.floatVal != 0.2f)
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* check the non-float case of an extension */
  sTest_Print("   testing non-float decimal(extension)..", out);
  if (aStream_WriteLine(ioLibRef, tokenStream, "test.txt", NULL))
    return aTrue;
  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkIdentifier)
    return aTrue;
  if (aStringCompare(pToken->v.identifier, "test"))
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkSpecial)
    return aTrue;
  if (pToken->v.special != '.')
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkIdentifier)
    return aTrue;
  if (aStringCompare(pToken->v.identifier, "txt"))
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* check large string handling */
  sTest_Print("   testing large strings..", out);
  if (aStream_WriteLine(ioLibRef, tokenStream, 
    "\"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghigklmnopqrstuvwxyz\"", NULL))
    return aTrue;
  if (aTokenizer_Next(ioLibRef, tokenizerRef, &pToken, NULL))
    return aTrue;
  if (pToken->eType != tkString)
    return aTrue;
  if (aStringCompare(pToken->v.string, 
  		     "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghigklmnopqrstuvwxyz"))
    return aTrue;
  if (aTokenizer_Dispose(ioLibRef, tokenizerRef, pToken, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   cleaning up..", out);
  if (aTokenizer_Destroy(ioLibRef, tokenizerRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  return(aFalse);

} /* sTest_aTokenizer_Next */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aFileTests
 */

aBool aFileTests(aIOLib ioLibRef, aStreamRef out)
{
  aStream_WriteLine(ioLibRef, out, " Basic File Tests", NULL);

  if (sTest_aFile_Open(ioLibRef, out))
    return 1;
  if (sTest_aFile_Close(ioLibRef, out))
    return 1;
  if (sTest_aFile_Read(ioLibRef, out))
    return 1;
  if (sTest_aFile_Write(ioLibRef, out))
    return 1;
  if (sTest_aFile_Seek(ioLibRef, out))
    return 1;
  if (sTest_aFile_GetSize(ioLibRef, out))
    return 1;
  if (sTest_aFile_Delete(ioLibRef, out))
    return 1;
  if (sTest_aFile_Use(ioLibRef, out))
    return 1;

  aStream_WriteLine(ioLibRef, out, " File Tests Complete", NULL);

  return(0);

} /* aFileTests */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDirectoryTests
 */

typedef struct {
  int 	count;
  aErr 	status;
  aStreamRef out;
} listProcData;

static aErr listProc(const char* pFileName,
		     const unsigned long ulSize,
		     void* vpRef)
{
  listProcData* pData = (listProcData*)vpRef;
  /* filenames are testX.foo (one-based) */
  unsigned long expected = (unsigned long)((pFileName[4] - '0') * 10);

  sTest_Print(pFileName, pData->out);
  sTest_Print(",", pData->out);
  pData->count++;

  if (ulSize != expected) {
    char num[10];
    aStringFromInt(num, ulSize);
    sTest_Print("size was wrong got ", pData->out);
    sTest_Print(num, pData->out);
    aStringFromInt(num, expected);
    sTest_Print(" instead of ", pData->out);
    sTest_Print(num, pData->out);
    return aErrIO;
  }

  return aErrNone;
}


aBool aDirectoryTests(aIOLib ref, aStreamRef out)
{
  aErr err;
  aFileRef f;
  listProcData d;

  aStream_WriteLine(ref, out, " Basic Directory Tests", NULL);


  sTest_Print("  NULL lib...", out);
  if (!aDirectory_List(NULL, aFileAreaPlugin, "", listProc,
  		       &d, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ref, out, "passed", NULL);


  sTest_Print("  NULL Callback...", out);
  if (!aDirectory_List(ref, aFileAreaPlugin, "", NULL,
  		       &d, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ref, out, "passed", NULL);



  sTest_Print("  NULL Extension...", out);
  if (!aDirectory_List(ref, aFileAreaPlugin, NULL, listProc,
  		       &d, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ref, out, "passed", NULL);



  sTest_Print("  Populating aPlugin Directory...", out);
  if (aFile_Open(ref, "test1.foo", 
  		 aFileModeWriteOnly, aFileAreaPlugin, &f, NULL))
    return aTrue;
  if (aFile_Write(ref, f, "0123456789", 10, NULL, NULL))
    return aTrue;
  if (aFile_Close(ref, f, NULL))
    return aTrue;
  if (aFile_Open(ref, "test2.foo", 
  		 aFileModeWriteOnly, aFileAreaPlugin, &f, NULL))
    return aTrue;
  if (aFile_Write(ref, f, "0123456789", 10, NULL, NULL))
    return aTrue;
  if (aFile_Write(ref, f, "0123456789", 10, NULL, NULL))
    return aTrue;
  if (aFile_Close(ref, f, NULL))
    return aTrue;
  aStream_WriteLine(ref, out, "passed", NULL);



  sTest_Print("  Enumerating Directory...", out);
  d.count = 0;
  d.status = aErrNone;
  d.out = out;
  if (aDirectory_List(ref, aFileAreaPlugin, ".foo", listProc,
  		      &d, &err))
    return aTrue;
  if (d.count != 2)
    return aTrue;
  if (d.status != aErrNone)
    return aTrue;
  aStream_WriteLine(ref, out, "passed", NULL);



  sTest_Print("  Cleaning Up aPlugin Directory...", out);
  if (aFile_Delete(ref, "test1.foo", aFileAreaPlugin, NULL))
    return aTrue;
  if (aFile_Delete(ref, "test2.foo", aFileAreaPlugin, NULL))
    return aTrue;
  aStream_WriteLine(ref, out, "passed", NULL);

  		 
  aStream_WriteLine(ref, out, " Directory Tests complete", NULL);

  return(0);

} /* aDirectoryTests */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStreamTests
 */

aBool aStreamTests(aIOLib ioLibRef, aStreamRef out)
{
  char msg[100];
  aStreamRef streamRef;
  aErr err = aErrNone;
  char* pBuffer;
  aMemSize size;
  char cur[3];
  char i;
  char readbuf[100];

  aStream_WriteLine(ioLibRef, out, " Basic Stream Tests", NULL);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   *
   *   aStream_Readline
   *
   */

  aStream_WriteLine(ioLibRef, out, "  aStream_ReadLine tests", NULL);

  aStringCopy(msg, "   Read lines in Windows test file..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  if (aStream_CreateFileInput(ioLibRef, "win32_testfile.txt", 
  			      aFileAreaTest,
			      &streamRef, &err))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Windows Test Data File"))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Line 2"))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, ""))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Line 4"))
    return aTrue;
  if (aStream_Destroy(ioLibRef, streamRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   Read lines in MacOS test file..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStream_CreateFileInput(ioLibRef, "mac_testfile.txt", 
  			      aFileAreaTest, &streamRef, &err))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Mac Test Data File"))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Line 2"))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, ""))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Line 4"))
    return aTrue;
  if (aStream_Destroy(ioLibRef, streamRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   Read lines in Unix test file..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStream_CreateFileInput(ioLibRef, "unix_testfile.txt", 
  			      aFileAreaTest, &streamRef, &err))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Unix Test Data File"))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Line 2"))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, ""))
    return aTrue;
  if (aStream_ReadLine(ioLibRef, streamRef, readbuf, 100, &err))
    return aTrue;
  if (aStringCompare(readbuf, "Line 4"))
    return aTrue;
  if (aStream_Destroy(ioLibRef, streamRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /*****************/
  /* Stream Buffer */
  /*****************/

  aStream_WriteLine(ioLibRef, out, "  Stream Buffer tests:", NULL);

  aStringCopy(msg, "   CreateBuffer creation..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStreamBuffer_Create(ioLibRef, 10, &streamRef, &err))
    return aTrue;
  if (aStream_Destroy(ioLibRef, streamRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  streamRef = NULL;

  aStringCopy(msg, "   CreateBuffer creation (NULL err)..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStreamBuffer_Create(ioLibRef, 10, &streamRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   CreateBuffer destroy (NULL err)..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStream_Destroy(ioLibRef, streamRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   CreateBuffer create bad lib ref..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aStreamBuffer_Create(NULL, 10, &streamRef, NULL))
    return aTrue;
  if (!aStreamBuffer_Create(NULL, 10, &streamRef, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   CreateBuffer bad size..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aStreamBuffer_Create(ioLibRef, 0, &streamRef, NULL))
    return aTrue;
  if (!aStreamBuffer_Create(ioLibRef, 0, &streamRef, &err))
    return aTrue;
  if (err != aErrRange)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   CreateBuffer create bad stream ref..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aStreamBuffer_Create(ioLibRef, 10, NULL, NULL))
    return aTrue;
  if (!aStreamBuffer_Create(ioLibRef, 10, NULL, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   CreateBuffer bad lib..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (!aStream_Destroy(NULL, streamRef, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  streamRef = NULL;
  pBuffer = 0;
  aStringCopy(msg, "   GetBuffer (empty)..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStreamBuffer_Create(ioLibRef, 10, &streamRef, &err))
    return aTrue;
  if (aStreamBuffer_Get(ioLibRef, streamRef, 
  			&size, &pBuffer, NULL))
    return aTrue;
  if (aStreamBuffer_Get(ioLibRef, streamRef, 
  			&size, &pBuffer, &err))
    return aTrue;
  if (size != 0)
    return aTrue;
  if (aStream_Destroy(ioLibRef, streamRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  streamRef = NULL;
  pBuffer = 0;
  aStringCopy(msg, "   Basic Buffer I/O..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);
  if (aStreamBuffer_Create(ioLibRef, 10, &streamRef, &err))
    return aTrue;
  /* start with some bytes in the buffer */
  cur[0] = 0; 
  cur[1] = 1; 
  cur[2] = 2;
  if (aStream_Write(ioLibRef, streamRef, cur, 3, &err))
    return aTrue;
  /* check the size */
  if (aStreamBuffer_Get(ioLibRef, streamRef, 
  			&size, &pBuffer, NULL))
    return aTrue;
  if (size != 3)
    return aTrue;
  if ((pBuffer[0] != 0) ||
      (pBuffer[1] != 1) ||
      (pBuffer[2] != 2))
    return aTrue;

  /* try a bunch of reads and writes to roll through the buffer */
  for (i = 3; i < 30; i += 3) {
    cur[0] = i;
    cur[1] = (char)(i + 1); 
    cur[2] = (char)(i + 2);
    if (aStream_Write(ioLibRef, streamRef, cur, 3, &err))
      return aTrue;
    if (aStream_Read(ioLibRef, streamRef, cur, 3, &err))
      return aTrue;
    if ((cur[0] != i - 3) 
    	|| (cur[1] != i - 2) 
    	|| (cur[2] != i - 1))
      return aTrue;
  }
  /* there should be three bytes left */
  if (aStreamBuffer_Get(ioLibRef, streamRef, 
  			&size, &pBuffer, NULL))
    return aTrue;
  if (size != 3)
    return aTrue;
  /* read out the final bytes */
  if (aStream_Read(ioLibRef, streamRef, cur, 3, &err))
    return aTrue;
  if ((cur[0] != 27) 
      || (cur[1] != 28) 
      || (cur[2] != 29))
    return aTrue;

  /* there should be nothing left */
  if (aStreamBuffer_Get(ioLibRef, streamRef, 
  			&size, &pBuffer, NULL))
    return aTrue;
  if (size != 0)
    return aTrue;
  if (aStream_Destroy(ioLibRef, streamRef, &err))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


#if defined(aWIN) || defined(aWINCE)
  aStream_WriteLine(ioLibRef, out, "  aStream_CreateSerial tests", NULL);

  aStringCopy(msg, "   Try a bad name..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  if (!aStream_CreateSerial(ioLibRef, 
#ifdef aWIN
			   "CIM2",
#else
			   "CIM2:",
#endif			   
			   9600,
			   &streamRef, 
			   &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStringCopy(msg, "   Try another bad name..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  if (!aStream_CreateSerial(ioLibRef, 
#ifdef aWIN
			   "COM2a3",
#else
			   "COM2a3:",
#endif			   
			   9600,
			   &streamRef, 
			   &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  aStringCopy(msg, "   Try a bad baudrate..");
  aStream_Write(ioLibRef, out, msg, aStringLen(msg), NULL);

  if (!aStream_CreateSerial(ioLibRef, 
#ifdef aWIN
			   "COM1",
#else
			   "COM1:",
#endif			   
			   9601,
			   &streamRef, 
			   &err))
    return aTrue;
  if (err != aErrRange)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

#endif /* aWIN and aWINCE */

  aStream_WriteLine(ioLibRef, out, " Stream Tests Complete", NULL);

  return aFalse;

} /* aStreamTests */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aZLibFilterTests
 */

#define BIGTEST 100000

aBool aZLibFilterTests(aIOLib ioLibRef, aStreamRef out)
{
  aErr err = aErrNone;
  aStreamRef input;
  aStreamRef output;
  aStreamRef stream;
  int i;
  char data[BIGTEST];
  char test[BIGTEST];

  aStream_WriteLine(ioLibRef, out, " ZLib Filter Stream Tests", NULL);

  aStream_WriteLine(ioLibRef, out, "  aStream_CreateZLibFilter tests", NULL);


  sTest_Print("   Create with NULL streamToFilter..", out);
  if (!aStream_CreateZLibFilter(ioLibRef, NULL, aFileModeWriteOnly, &stream, NULL))
    return aTrue;
  if (!aStream_CreateZLibFilter(ioLibRef, NULL, aFileModeWriteOnly, &stream, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   Create with NULL filtered stream pointer..", out);
  if (aStream_CreateFileOutput(ioLibRef, "zlibtest.gz", aFileAreaTest, &output, NULL))
    return aTrue;
  if (!aStream_CreateZLibFilter(ioLibRef, output, aFileModeWriteOnly, NULL, NULL))
    return aTrue;
  if (!aStream_CreateZLibFilter(ioLibRef, output, aFileModeWriteOnly, NULL, &err))
    return aTrue;
  if (err != aErrParam)
    return aTrue;
  if (aStream_Destroy(ioLibRef, output, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   Create with output file and read mode..", out);
  if (aStream_CreateFileOutput(ioLibRef, "zlibtest.gz", aFileAreaTest, &output, NULL))
    return aTrue;
  if (!aStream_CreateZLibFilter(ioLibRef, output, aFileModeReadOnly, &stream, NULL))
    return aTrue;
  if (!aStream_CreateZLibFilter(ioLibRef, output, aFileModeReadOnly, &stream, &err))
    return aTrue;
  if (err != aErrMode)
    return aTrue;
  if (aStream_Destroy(ioLibRef, output, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   Create with input file and write mode..", out);
  if (aStream_CreateFileInput(ioLibRef, "zlibtest.gz", aFileAreaTest, &input, NULL))
    return aTrue;
  if (!aStream_CreateZLibFilter(ioLibRef, input, aFileModeWriteOnly, &stream, NULL))
    return aTrue;
  if (!aStream_CreateZLibFilter(ioLibRef, input, aFileModeWriteOnly, &stream, &err))
    return aTrue;
  if (err != aErrMode)
    return aTrue;
  if (aStream_Destroy(ioLibRef, input, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   Create a simple file with \"zlib rules\\r\\n\" 100 times..", out);
  if (aStream_CreateFileOutput(ioLibRef, "zlibtest.gz", aFileAreaTest, &output, NULL))
    return aTrue;
  if (aStream_CreateZLibFilter(ioLibRef, output, aFileModeWriteOnly, &stream, &err))
    return aTrue;
  if (err != aErrNone)
    return aTrue;
  for (i = 0; i < 100; i++) {
    if (aStream_Write(ioLibRef, stream, "zlib rules\r\n", 12, NULL))
      return aTrue;
  }
  if (aStream_Destroy(ioLibRef, stream, NULL))
    return aTrue;
  if (aStream_Destroy(ioLibRef, output, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   Read the simple file with \"zlib rules\\r\\n\" 100 times..", out);
  if (aStream_CreateFileInput(ioLibRef, "zlibtest.gz", aFileAreaTest, &input, NULL))
    return aTrue;
  if (aStream_CreateZLibFilter(ioLibRef, input, aFileModeReadOnly, &stream, &err))
    return aTrue;
  if (err != aErrNone)
    return aTrue;
  for (i = 0; i < 100; i++) {
    if (aStream_ReadLine(ioLibRef, stream, data, 11, NULL))
      return aTrue;
    if (aStringCompare(data, "zlib rules"))
      return aTrue;
  }
  if (aStream_Destroy(ioLibRef, stream, NULL))
    return aTrue;
  if (aStream_Destroy(ioLibRef, input, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   write and read a 100,000 character random file..", out);
  
  for (i = 0; i < BIGTEST; i++)
    data[i] = (char)rand();
  
  if (aStream_CreateFileOutput(ioLibRef, "zlibtest.gz", aFileAreaTest, &output, NULL))
    return aTrue;
  if (aStream_CreateZLibFilter(ioLibRef, output, aFileModeWriteOnly, &stream, &err))
    return aTrue;
  if (err != aErrNone)
    return aTrue;
  if (aStream_Write(ioLibRef, stream, data, BIGTEST, NULL))
    return aTrue;
  if (aStream_Destroy(ioLibRef, stream, NULL))
    return aTrue;
  if (aStream_Destroy(ioLibRef, output, NULL))
    return aTrue;
  if (aStream_CreateFileInput(ioLibRef, "zlibtest.gz", aFileAreaTest, &input, NULL))
    return aTrue;
  if (aStream_CreateZLibFilter(ioLibRef, input, aFileModeReadOnly, &stream, &err))
    return aTrue;
  if (err != aErrNone)
    return aTrue;
  if (aStream_Read(ioLibRef, stream, test, BIGTEST, NULL))
      return aTrue;
  for (i = 0; i < BIGTEST; i++) {
    if (test[i] != data[i])
      return aTrue;
  }
  if (aStream_Destroy(ioLibRef, stream, NULL))
    return aTrue;
  if (aStream_Destroy(ioLibRef, input, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  aStream_WriteLine(ioLibRef, out, " ZLib Filter Stream Tests Complete", NULL);
  
  return aFalse;

} /* aZLibFilterTests */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMoreIOTests
 */

typedef struct aMoreIOTest {
  aIOLib ioRef;
  aStreamRef out;
  aFileArea eArea;
  aMemSize nBufSize;
  char* pBuf;
} aMoreIOTest;

static aErr sMoreIODirProc (
  const char* pFilename,
  const unsigned long nSize,
  void* ref
);
aErr sMoreIODirProc (
  const char* pFilename,
  const unsigned long nSize,
  void* ref
)
{
  aErr err = aErrNone;
  aMoreIOTest* pData = (aMoreIOTest*)ref;
  aFileRef in;
  unsigned long nRead;
  aStreamRef rawBuffer;
  aStreamRef zlibFilter;
  aStreamRef temp;
  aMemSize size;
  char* pBuf;
  char num[10];

  /* hack to avoid the CVS directory in the build tree */
  if (!aStringCompare(pFilename, "CVS"))
    return aErrNone;
  
  sTest_Print("    ", pData->out);
  sTest_Print(pFilename, pData->out);
  
  /* grow the buffer if needed */
  if (err == aErrNone) {
    if (nSize > pData->nBufSize) {
      if (pData->pBuf)
        aMemFree(pData->pBuf);
      pData->pBuf = (char*)aMemAlloc(nSize);
      if (pData->pBuf)
        pData->nBufSize = nSize;
      else
        err = aErrMemory;
    }
  }
  
  /* open the file to read it in */
  if (err == aErrNone)
    aFile_Open(pData->ioRef, pFilename, aFileModeReadOnly, pData->eArea, &in, &err);

  /* read the file into the buffer */
  if (err == aErrNone) {
    aFile_Read(pData->ioRef, in, pData->pBuf, nSize, &nRead, &err);
    if (nRead != nSize)
      err = aErrIO;
  }

  /* close the file */
  if (err == aErrNone)
    aFile_Close(pData->ioRef, in, &err);

  sTest_Print(".", pData->out);

  /* now, open a new buffer stream */
  if (err == aErrNone)
    aStreamBuffer_Create(pData->ioRef, 100, &rawBuffer, &err);

  /* now, create a zlib filter on the buffer */
  if (err == aErrNone)
    aStream_CreateZLibFilter(pData->ioRef, rawBuffer, aFileModeWriteOnly, &zlibFilter, &err);

  /* write the file contents to the filtered buffer */
  if (err == aErrNone)
    aStream_Write(pData->ioRef, zlibFilter, pData->pBuf, nSize, &err);
  
  /* destroy the zlib filter */
  if (err == aErrNone)
    aStream_Destroy(pData->ioRef, zlibFilter, &err);

  sTest_Print(".", pData->out);

  /* now, build an output file to save the data into */
  if (err == aErrNone)
    aStream_CreateFileOutput(pData->ioRef, "temp.gz", aFileAreaObject, &temp, &err);

  /* flush the stream to the temp */
  if (err == aErrNone)
    aStreamBuffer_Flush(pData->ioRef, rawBuffer, temp, &err);

  /* destroy the buffer stream */
  if (err == aErrNone)
    aStream_Destroy(pData->ioRef, rawBuffer, &err);

  /* destroy the temp file stream */
  if (err == aErrNone)
    aStream_Destroy(pData->ioRef, temp, &err);

  sTest_Print(".", pData->out);

  /* get the file size to compute compression */
  if (err == aErrNone)
    aFile_Open(pData->ioRef, "temp.gz", aFileModeReadOnly, aFileAreaObject, &in, &err);
  if (err == aErrNone)
    aFile_GetSize(pData->ioRef, in, &nRead, &err);
  if (err == aErrNone)
    aFile_Close(pData->ioRef, in, &err);
  aStringFromInt(num, (int)(((float)nRead / (float)nSize) * 100.0f));

  sTest_Print(num, pData->out);
  sTest_Print("%", pData->out);

  /* read the file back in */
  if (err == aErrNone)
    aStream_CreateFileInput(pData->ioRef, "temp.gz", aFileAreaObject, &temp, &err);

  /* now, create a zlib filter on the file */
  if (err == aErrNone)
    aStream_CreateZLibFilter(pData->ioRef, temp, aFileModeReadOnly, &zlibFilter, &err);

  /* read it into a buffer */
  if (err == aErrNone)
    aStreamBuffer_Create(pData->ioRef, 100, &rawBuffer, &err);

  /* read the temp into a buffer */
  if (err == aErrNone)
    aStream_Flush(pData->ioRef, zlibFilter, rawBuffer, &err);

  /* destroy the zlib filter */
  if (err == aErrNone)
    aStream_Destroy(pData->ioRef, zlibFilter, &err);

  /* destroy the temp file stream */
  if (err == aErrNone)
    aStream_Destroy(pData->ioRef, temp, &err);

  sTest_Print(".", pData->out);

  /* get a pointer to the file contents */
  if (err == aErrNone)
    aStreamBuffer_Get(pData->ioRef, rawBuffer, &size, &pBuf, &err);

  /* check the file sizes */
  if (size != nSize) {
    aStream_WriteLine(pData->ioRef, pData->out, " SIZE MISSMATCH!", NULL);
    err = aErrIO;
  }

  /* compare the file contents */
  if (err == aErrNone) {
    int i;
    char* p1 = pBuf;
    char* p2 = pData->pBuf;
    for (i = 0; (i < (int)nSize) && (err == aErrNone); i++) {
      if (*p1++ != *p2++) {
        aStream_WriteLine(pData->ioRef, pData->out, " DATA MISSMATCH!", NULL);
        err = aErrIO;
      }
    }
  }

  sTest_Print(".", pData->out);

  /* destroy the buffer stream */
  if (err == aErrNone)
    aStream_Destroy(pData->ioRef, rawBuffer, &err);
    
  /* clean up the temp file */
  if (err == aErrNone)
    aFile_Delete(pData->ioRef, "temp.gz", aFileAreaObject, &err);

  sTest_Print(".", pData->out);

  if (err == aErrNone)
    aStream_WriteLine(pData->ioRef, pData->out, " passed", &err);
  
  return err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aMoreIOTests
 */

aBool aMoreIOTests(aIOLib ioLibRef, aStreamRef out)
{
  aErr err = aErrNone;
  aMoreIOTest data;
  
  data.ioRef = ioLibRef;
  data.out = out;
  data.eArea = aFileAreaTest;
  data.nBufSize = 0;
  data.pBuf = 0;

  aStream_WriteLine(ioLibRef, out, " More IO Tests", NULL);

  aStream_WriteLine(ioLibRef, out, "  aGenaral IO test on aTest files", NULL);

  sTest_Print("   Walking directory..", out);
  if (aDirectory_List(ioLibRef, data.eArea, "", sMoreIODirProc, &data, &err))
    return aTrue;
  if (err != aErrNone)
    return aTrue;  
  if (data.pBuf)
    aMemFree(data.pBuf);

  aStream_WriteLine(ioLibRef, out, "  completed all files in test", NULL);

  aStream_WriteLine(ioLibRef, out, " More IO Tests Complete", NULL);

  return aFalse;

} /* aMoreIOTests */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSettingFile tests
 */

aBool aSettingFileTests(aIOLib ioLibRef, aStreamRef out)
{
  aErr err = aErrNone;
  aSettingFileRef settingFileRef;
  aStreamRef testFileStream;
  int intVal;
  unsigned long ulongVal;
  float floatVal;
  char* stringVal;

  aStream_WriteLine(ioLibRef, out, " Setting File Tests", NULL);

  aStream_WriteLine(ioLibRef, out, "  aSettingFile_Create tests", NULL);

  sTest_Print("   Create with NULL ioLib..", out);
  if (!aSettingFile_Create(NULL, 32, "test.config", 
  			   NULL, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Create with NULL ioLib, ", out);
  if (!aSettingFile_Create(NULL, 32, "test.config", 
  			   NULL, &err))
    return aTrue;
  sTest_Print("checking error..", out);
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   Create with NULL settingFileRef..", out);
  if (!aSettingFile_Create(ioLibRef, 32, "test.config", 
  			   NULL, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Create with NULL settingFileRef, ", out);
  if (!aSettingFile_Create(ioLibRef, 32, "test.config", 
  			   NULL, &err))
    return aTrue;
  sTest_Print("checking error..", out);
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  sTest_Print("   Create with NULL filename..", out);
  if (!aSettingFile_Create(ioLibRef, 32, NULL, 
  			   &settingFileRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Create with NULL ioLib, ", out);
  if (!aSettingFile_Create(NULL, 32, "foo.config", 
  			   &settingFileRef, &err))
    return aTrue;
  sTest_Print("checking error..", out);
  if (err != aErrParam)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Create without setting file..", out);
  if (aSettingFile_Create(ioLibRef, 32, "foo.config", 
  			  &settingFileRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get default int, ", out);
  if (aSettingFile_GetInt(ioLibRef, settingFileRef, "empty_int", 
  			  &intVal, 32, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (intVal != 32)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get default ulong, ", out);
  if (aSettingFile_GetULong(ioLibRef, settingFileRef, "empty_ulong", 
  			    &ulongVal, 1000000, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (ulongVal != 1000000)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get default float, ", out);
  if (aSettingFile_GetFloat(ioLibRef, settingFileRef, "empty_float", 
  			    &floatVal, 3.1459f, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (floatVal != 3.1459f)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get default string, ", out);
  if (aSettingFile_GetString(ioLibRef, settingFileRef, "empty_string", 
  			     &stringVal, "not there", NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (aStringCompare(stringVal, "not there"))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  sTest_Print("   Destroy..", out);
  if (aSettingFile_Destroy(ioLibRef, settingFileRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Destroying again..", out);
  if (!aSettingFile_Destroy(ioLibRef, settingFileRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Destroying again, ", out);
  if (!aSettingFile_Destroy(ioLibRef, settingFileRef, &err))
    return aTrue;
  sTest_Print("checking error..", out);
  if (err != aErrParam)
    return aTrue;  
  aStream_WriteLine(ioLibRef, out, "passed", NULL);


  /* here we build a small setting file for testing */
  sTest_Print("   Building test file..", out);
  if (aStream_CreateFileOutput(ioLibRef, "test.config",
  			       aFileAreaBinary, &testFileStream, NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, "# test file", NULL))
    return aTrue;

  if (aStream_WriteLine(ioLibRef, testFileStream, "intVal1 = 56", NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, "intVal2=0", NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, "intVal3=0xF333", NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, "intVal4=0x0", NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, "intVal5=-1", NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, "intVal5=100", NULL))
    return aTrue;

  if (aStream_WriteLine(ioLibRef, testFileStream, "ulongVal1 = 2000000", NULL))
    return aTrue;

  if (aStream_WriteLine(ioLibRef, testFileStream, "floatVal1 = 3.4", NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, "floatVal2=7", NULL))
    return aTrue;

  if (aStream_WriteLine(ioLibRef, testFileStream, "stringVal1=test string", NULL))
    return aTrue;
  if (aStream_WriteLine(ioLibRef, testFileStream, 
  			"trunc = 012345678901234567890123456789012345", 
  			NULL))
    return aTrue;
  if (aStream_Destroy(ioLibRef, testFileStream, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Create with setting file..", out);
  if (aSettingFile_Create(ioLibRef, 32, "test.config", 
  			  &settingFileRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get default int, ", out);
  if (aSettingFile_GetInt(ioLibRef, settingFileRef, "empty_int", 
  			  &intVal, 32, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (intVal != 32)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get existing int, ", out);
  if (aSettingFile_GetInt(ioLibRef, settingFileRef, "intVal1", 
  			  &intVal, 32, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (intVal != 56)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  sTest_Print("   Get existing int, ", out);
  if (aSettingFile_GetInt(ioLibRef, settingFileRef, "intVal2", 
			  &intVal, 33, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (intVal != 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  sTest_Print("   Get existing int, ", out);
  if (aSettingFile_GetInt(ioLibRef, settingFileRef, "intVal3", 
			  &intVal, 33, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (intVal != 0xF333)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  sTest_Print("   Get existing int, ", out);
  if (aSettingFile_GetInt(ioLibRef, settingFileRef, "intVal4", 
			  &intVal, 33, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (intVal != 0)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  sTest_Print("   Get existing int, ", out);
  if (aSettingFile_GetInt(ioLibRef, settingFileRef, "intVal5", 
			  &intVal, 33, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (intVal != -1)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  
  sTest_Print("   Get default ulong, ", out);
  if (aSettingFile_GetULong(ioLibRef, settingFileRef, "empty_ulong", 
  			    &ulongVal, 1000000, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (ulongVal != 1000000)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get existing ulong, ", out);
  if (aSettingFile_GetULong(ioLibRef, settingFileRef, "ulongVal1", 
  			    &ulongVal, 1000000, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (ulongVal != 2000000)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get default float, ", out);
  if (aSettingFile_GetFloat(ioLibRef, settingFileRef, "empty_float", 
  			    &floatVal, 3.1459f, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (floatVal != 3.1459f)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get existing float, ", out);
  if (aSettingFile_GetFloat(ioLibRef, settingFileRef, "floatVal1", 
  			    &floatVal, 3.1459f, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (floatVal != 3.4f)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get existing float, ", out);
  if (aSettingFile_GetFloat(ioLibRef, settingFileRef, "floatVal2", 
  			    &floatVal, 3.1459f, NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (floatVal != 7.0f)
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  sTest_Print("   Get default string, ", out);
  if (aSettingFile_GetString(ioLibRef, settingFileRef, "empty_string", 
  			     &stringVal, "not there", NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (aStringCompare(stringVal, "not there"))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get existing string, ", out);
  if (aSettingFile_GetString(ioLibRef, settingFileRef, "stringVal1", 
  			     &stringVal, "not there", NULL))
    return aTrue;
  sTest_Print("checking..", out);
  if (aStringCompare(stringVal, "test string"))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Get truncated string, ", out);
  if (aSettingFile_GetString(ioLibRef, settingFileRef, "trunc", 
  			     &stringVal, "not there", NULL))
    return aTrue;
  sTest_Print("checking..", out);
  /* should be truncated to 31 characters to handle the terminator */
  if (aStringCompare(stringVal, "0123456789012345678901234567890"))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);
  
  sTest_Print("   Destroy..", out);
  if (aSettingFile_Destroy(ioLibRef, settingFileRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Destroying again..", out);
  if (!aSettingFile_Destroy(ioLibRef, settingFileRef, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  sTest_Print("   Destroying again, ", out);
  if (!aSettingFile_Destroy(ioLibRef, settingFileRef, &err))
    return aTrue;
  sTest_Print("checking error..", out);
  if (err != aErrParam)
    return aTrue;  
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  /* here we build a small setting file for testing */
  sTest_Print("   Cleaning up test file..", out);
  if (aFile_Delete(ioLibRef, "test.config",
  		   aFileAreaBinary, NULL))
    return aTrue;
  aStream_WriteLine(ioLibRef, out, "passed", NULL);

  return aFalse;

} /* aSettingFileTests */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aBool aIOTests(aIOLib ioLibRef, aStreamRef outputStream)
{
  aErr e;
  unsigned long addr;
  char textAddr[16];

  aStream_WriteLine(ioLibRef, outputStream, 
  		    " Basic General IO Tests", NULL);

  sTest_Print("   Getting IP address: ", outputStream);
  if (aIO_GetInetAddr(ioLibRef, &addr, &e))
    return aTrue;
  aUtil_FormatInetAddr(textAddr, addr, NULL);
  sTest_Print(textAddr, outputStream);
  aStream_WriteLine(ioLibRef, outputStream, " passed", NULL);

  if (sTest_aIO_GetMSTicks(ioLibRef, outputStream))
    return aTrue;

  if (sTest_aIO_MSSleep(ioLibRef, outputStream))
    return aTrue;

  aStream_WriteLine(ioLibRef, outputStream, 
  		    " General IO tests complete", NULL);

  return aFalse;

} /* aIOTests */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aBool aTokenizerTests(aIOLib ioLibRef, aStreamRef out)
{
  aStream_WriteLine(ioLibRef, out, 
  		    " Basic Tokenizer Tests", NULL);

  if (sTest_aTokenizer_Create(ioLibRef, out))
    return aTrue;
  if (sTest_aTokenizer_Next(ioLibRef, out))
    return aTrue;

  aStream_WriteLine(ioLibRef, out, " Tokenizer tests complete", NULL);

  return aFalse;

} /* aTokenizerTests */
