/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aIO.c                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of file I/O routines for unix.      */
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

#ifdef aUNIX

#ifdef aMACX
#include <Carbon/Carbon.h>
#endif /* aMACX */

#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>

#include "unix_aIO.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIO_GetLibRef
 */

aLIBRETURN
aIO_GetLibRef(aIOLib* pRef,			
	      aErr* pErr)
{
  aErr err = aErrNone;
  aUnixIOLib* pLib = NULL;
  
  if (pRef == NULL)
    err = aErrParam;
  
  if (err == aErrNone) {
    pLib = (aUnixIOLib*)aMemAlloc(sizeof(aUnixIOLib));
    if (pLib == NULL)
      err = aErrMemory;
  }

  if (err == aErrNone) {
    aBZero(pLib, sizeof(aUnixIOLib));
    err = aIOInternal_Initialize((aIO*)pLib);
  }
 
  if (err == aErrNone)
    *pRef = pLib;

  if (pErr != NULL)
    *pErr = err;

  return err;

} /* end of aIO_GetLibRef routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIO_ReleaseLibRef
 */

aLIBRETURN 
aIO_ReleaseLibRef(aIOLib libRef,
		  aErr* pErr)
{
  aErr err = aErrNone;

  aVALIDIO(libRef);

  if (err == aErrNone) {
    aMemFree((aUnixIOLib*)libRef);
  }

  if (pErr != NULL)
    *pErr = err;
  
  return err;

} /* end of aIO_ReleaseLibRef routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIO_GetMSTicks
 */

aLIBRETURN aIO_GetMSTicks(aIOLib ioRef,
			  unsigned long* pNTicks,
			  aErr* pErr)
{
  aErr err = aErrNone;
  struct timeval ts;
  struct timezone tz;

  aVALIDIO(ioRef);

  if ((err == aErrNone) && (pNTicks == NULL))
    err = aErrParam;

  if (err == aErrNone) {
    if (gettimeofday(&ts, &tz)) {
      err = aErrIO;
    } else {
      /* here we scale down the seconds by 10000 to keep within a
       * long */
      *pNTicks = (unsigned long)
      	((ts.tv_sec % 10000) * 1000L + ts.tv_usec / 1000L);
    }
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aIO_GetMSTicks */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIO_MSSleep
 */

aLIBRETURN aIO_MSSleep(aIOLib ioRef,
		       const unsigned long msTime,
		       aErr* pErr)
{
  aErr err = aErrNone;
  unsigned long ulTime;

  if ((err == aErrNone) 
      && (msTime > 0)) {

    /* time is specified in microseconds, we want millis */
    ulTime = msTime * 1000;

    /* usleep has a maximum of 1000000 so we break it up here */
    while ((err == aErrNone) && (ulTime > 0)) {
      unsigned long inc;
      if (ulTime > 999999)
        inc = 999999;
      else
        inc = ulTime;
      ulTime -= inc;

      /* usleep doesn't return an error on all unix platforms */
      usleep(inc);
    }
  }

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aIO_MSSleep */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aIO_GetInetAddr
 */

aLIBRETURN aIO_GetInetAddr(aIOLib ioRef,
		           unsigned long* pAddress,
		           aErr* pErr)
{
  aErr err = aErrNone;

  aVALIDIO(ioRef);
  if ((err == aErrNone) && !pAddress)
    err = aErrParam;

#ifndef HOST_NAME_MAX
#define HOST_NAME_MAX 256
#endif /* HOST_NAME_MAX */

  {
    char name[HOST_NAME_MAX];

    /* get the host name */
    if (err == aErrNone) {
      if (gethostname(name, HOST_NAME_MAX))
        err = aErrIO;
    }

    /* now find the ip address of the hostname */
    if (err == aErrNone) {
      unsigned long address = 0;
      struct hostent* pHostEnt = gethostbyname(name);
      aBool bLocalHost = aFalse;

      err = aErrIO;
      if (pHostEnt && (pHostEnt->h_length == 4)) {
        int i = 0;
        /* scan the addresses and take the first non-loopback entry */
        while ((err == aErrIO) && pHostEnt->h_addr_list[i]) {
          char* pNetAddr = pHostEnt->h_addr_list[i];
          address = htonl(*((unsigned long*)pNetAddr));
	  if (address != 0x7F000001) {
            *pAddress = address;
	    err = aErrNone;
          } else 
	    bLocalHost = aTrue;
          i++;
        } /* while */
      }

      /* fall back on the localhost if none other found */
      if ((err == aErrIO) && bLocalHost) {
	address = 0x7F000001;
	err = aErrNone;
      }
    }
  }

#undef HOST_NAME_MAX

  if (pErr != NULL)
    *pErr = err;

  return (aLIBRETURN)(err != aErrNone);

} /* aIO_MSSleep */


#endif /* aUNIX */
