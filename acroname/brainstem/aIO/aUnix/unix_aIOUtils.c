/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aIOUtils.c                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of I/O utilities for unix.          */
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

#include <unistd.h>

#include "aIO.h"
#include "unix_aIOUtils.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_aFileFullPath
 */

void unix_aFileFullPath(char* fullpath,
		        const char* pFilename,
		        const aFileArea eArea)
{
  /* initialize to start with */
  fullpath[0] = 0;

  /* Native filepaths are not managed at all and we just use them
   * as sent to us. */
  if (eArea == aFileAreaNative) {
    aStringCat(fullpath, pFilename);
    return;
  }

#ifdef aMACX
  /* MacX has a goofy concept of current working directory.
   * We need to find this out to do path relative work. */
  {
    getwd(fullpath);
    aStringCat(fullpath, "/");
  }
#endif

  /* binary is just where we are */
  if (eArea == aFileAreaBinary) {
    aStringCat(fullpath, "./");
    aStringCat(fullpath, pFilename);
    return;
  }

  aStringCat(fullpath, "../");
  
  switch (eArea) {
  case aFileAreaUser:
    aStringCat(fullpath, txtFileAreaUser);
    break;
  case aFileAreaSystem:
    aStringCat(fullpath, txtFileAreaSystem);
    break;
  case aFileAreaObject:
    aStringCat(fullpath, txtFileAreaObject);
    break;
  case aFileAreaBinary:
    aStringCat(fullpath, txtFileAreaBinary);
    break;
  case aFileAreaTest:
    aStringCat(fullpath, txtFileAreaTest);
    break;
  case aFileAreaInclude:
    aStringCat(fullpath, txtFileAreaInclude);
    break;
  case aFileAreaSource:
    aStringCat(fullpath, txtFileAreaSource);
    break;
  case aFileAreaAsset:
    aStringCat(fullpath, txtFileAreaAsset);
    break;
  case aFileAreaPlugin:
    aStringCat(fullpath, txtFileAreaPlugin);
    break;
  case aFileAreaSymonym:
    aStringCat(fullpath, txtFileAreaSymonym);
    break;
  case aFileAreaDocumentation:
    aStringCat(fullpath, txtFileAreaDocumentation);
    break;
  default:
    break;
  } /* switch */
  
  aStringCat(fullpath, "/");
  aStringCat(fullpath, pFilename);

} /* unix_aFileFullPath */

#endif /* aUNIX */
