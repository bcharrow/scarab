/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStream_STDIO_Console.c                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: cross-platform stdio console stream                */
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

#include <stdio.h>

#include "aStream_STDIO_Console.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * static local routines (callbacks)
 */

static aErr sSTDIO_ConsolePut(char* pData,
                              void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_Create_STDIO_Console_Output
 */

aErr aStream_Create_STDIO_Console_Output(aIOLib libRef, 
					 aStreamRef* pStreamRef)
{
  aErr err = aErrNone;
  aStreamRef streamRef;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  if (pStreamRef == NULL)
    return aErrParam;

  *pStreamRef = NULL;
  
  if (aStream_Create(libRef, 
  		     NULL,
  		     sSTDIO_ConsolePut,
  		     NULL,
  		     NULL,
  		     &streamRef, 
			 &err))
    return err;

  *pStreamRef = streamRef;
  
  return(err);

} /* aStream_Create_STDIO_Console_Output */ 


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSTDIO_ConsolePut
 */

static aErr sSTDIO_ConsolePut(char* pData,
			      void* ref)
{
  printf("%c", *pData);

  return(aErrNone);

} /* sSTDIO_ConsolePut */

