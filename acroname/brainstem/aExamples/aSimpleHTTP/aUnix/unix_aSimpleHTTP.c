/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aSimpleHTTP.c	                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Simple "Hello World" type application for the	   */
/*		aHTTP server using Unix.	      		   */
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

#include "aUI.h"	/* User Interface Library Header   */

/* hardwire the IP address and port */
#define PORT 80


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static aErr aSimpleHTTP_HandleRequest(const char* pURL,
				      aSymbolTableRef params,
				      aStreamRef reply,
				      void* vpRef)
{
  printf("requested %s\n", pURL);

  return aErrNotFound;

} /* aSimpleHTTP_HandleRequest */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int main(int argc, char* argv[]) {
  aErr err = aErrNone;
  aIOLib ioRef;
  aUILib uiRef;
  aHTTPRef http;
  aBool bDone = aFalse;

  /* Get the references to the aIO and aUI library objects. */
  if (err == aErrNone)
    aIO_GetLibRef(&ioRef, &err);
  if (err == aErrNone)
    aUI_GetLibRef(&uiRef, &err);

  aAssert(err == aErrNone);

  /* Build an http server object. */
  if (err == aErrNone)
    aHTTP_Create(uiRef, 
		 NULL,
		 aSimpleHTTP_HandleRequest,
		 &bDone,
		 &http, 
		 &err);

  printf("Please open a browser to \n  http://localhost:8000\n");
  printf("You should see a not-found error message.\n");
  printf("Each time you request it, you should see a message here\n");
  printf("Control-C will exit\n");

  /* loop while not done */
  while (!bDone && (err == aErrNone)) {
    aBool bChanged = aFalse;

    aHTTP_TimeSlice(uiRef, http, &bChanged, &err);

    aAssert(err == aErrNone);

    if ((err == aErrNone) && !bChanged)
      aIO_MSSleep(ioRef, 10, &err);
    aAssert(err == aErrNone);
    aAssert(!bDone);
  }

  /* release the libraries now that we are done whether there
   * were errors or not */
  aUI_ReleaseLibRef(uiRef, NULL);
  aIO_ReleaseLibRef(ioRef, NULL);

  return 0;

} /* main */
