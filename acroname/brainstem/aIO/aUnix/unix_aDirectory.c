/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aDirectory.h                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: header for I/O library routines for unix.          */
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

#include <sys/types.h>
#include <sys/dir.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/errno.h>

#include "aIO.h"
#include "aIOInternal.h"
#include "unix_aIOUtils.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local globals
 */

const char* gpExtension = NULL;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local static routines
 */

#ifdef aMACX
static int sFileSelect(struct direct* pEntry);
#else
static int sFileSelect(const struct direct* pEntry);
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sFileSelect
 */

#ifdef aMACX
int sFileSelect(struct direct* pEntry)
#else
int sFileSelect(const struct direct* pEntry)
#endif
{
  if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
    return 0;

  if (*gpExtension) {
    int nameLen = strlen(pEntry->d_name);
    int extLen = strlen(gpExtension);
    if (nameLen < extLen)
      return 0;
    if (strcmp(&pEntry->d_name[nameLen - extLen], gpExtension))
      return 0;
  }

  return 1;

} /* sFileSelect */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDirectory_List
 */

aLIBRETURN aDirectory_List(aIOLib ioRef,
			   const aFileArea eArea,
			   const char* pExtension,
			   aDirectoryListProc listProc,
			   void* vpRef,
			   aErr* pErr)
{
  aErr err = aErrNone;
  char fullpath[aMAXUNIXFILECHARS];

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check the parameters
   */
  if ((pExtension == NULL) || (ioRef == NULL) || (listProc == NULL))
    err = aErrParam;
  aVALIDIO(ioRef);

  if (err == aErrNone) {
    struct direct **files;
    int i, count;

    unix_aFileFullPath(fullpath, "", eArea);

    gpExtension = pExtension;
    count = scandir(fullpath, &files, sFileSelect, alphasort);
    gpExtension = NULL;

    strcat(fullpath, "/");
    for (i = 0; (err == aErrNone) && (i < count); i++) {
      struct stat fileInfo;
      unsigned long size;
      char filename[aMAXUNIXFILECHARS];
      strcpy(filename, fullpath);
      strcat(filename, files[i]->d_name);
      if (stat(filename, &fileInfo)) {
        err = aErrIO;
      } else {
        size = fileInfo.st_size;
        err = listProc(files[i]->d_name, size, vpRef);
      }
    }
  }

  if (pErr != NULL)
    *pErr = err; 

  return (aLIBRETURN)(err != aErrNone);
 
} /* aDirectory_List */

#endif /* aUNIX */
