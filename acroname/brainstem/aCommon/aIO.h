/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aIO.h                                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent I/O layer.    */
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

#ifndef _aIO_H_
#define _aIO_H_

#include "aErr.h"
#include "aOSDefs.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to the I/O library
 */

typedef aLIBREF aIOLib;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * define symbol import mechanism
 */

#ifndef aIO_EXPORT
#define aIO_EXPORT aLIB_IMPORT
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * I/O library manipulation routines
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

aIO_EXPORT aLIBRETURN
aIO_GetLibRef(aIOLib* pIORef, 
              aErr* pErr);

aIO_EXPORT aLIBRETURN 
aIO_ReleaseLibRef(aIOLib ioRef, 
		  aErr* pErr);

aIO_EXPORT aLIBRETURN 
aIO_GetVersion(aIOLib ioRef, 
	       unsigned long* pulVersion,
	       aErr* pErr);

#ifdef __cplusplus 
}
#endif /* __cplusplus */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * opaque reference to a file
 */

typedef void* aFileRef;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Serial Input buffer size.
 */

#define aSERIAL_INBUFSIZE     1024


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Maximum number of TCP/IP connections on a server socket
 */

#define aTCP_MAXCONNECT         10


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File I/O modes.
 */

typedef enum aFileMode {
  aFileModeReadOnly,
  aFileModeWriteOnly,
  aFileModeUnknown
} aFileMode;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File storage areas.
 */

typedef enum aFileArea {
  aFileAreaUser,
  aFileAreaSystem,
  aFileAreaObject,
  aFileAreaBinary,
  aFileAreaTest,
  aFileAreaInclude,
  aFileAreaSource,
  aFileAreaAsset,
  aFileAreaPlugin,
  aFileAreaSymonym,
  aFileAreaDocumentation,
  aFileAreaNative
} aFileArea;

#define txtFileAreaUser			"aUser"
#define txtFileAreaSystem		"aSystem"
#define txtFileAreaObject		"aObject"
#define txtFileAreaBinary		"aBinary"
#define txtFileAreaTest			"aTest"
#define txtFileAreaInclude		"aInclude"
#define txtFileAreaSource		"aSource"
#define txtFileAreaAsset		"aAsset"
#define txtFileAreaPlugin		"aPlugin"
#define txtFileAreaSymonym		"aSymonym"
#define txtFileAreaDocumentation	"aDocumentation"
#define txtFileAreaNative		"native"

#define nMAXFILEAREACHARS	14


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * File Routine definitions
 */

#ifdef __cplusplus
extern "C" {
#endif

aIO_EXPORT aLIBRETURN 
aFile_Open(aIOLib ioRef,
           const char* pFilename,
	   const aFileMode eMode,
	   const aFileArea eArea,
	   aFileRef* pFileRef,
	   aErr* pErr);

aIO_EXPORT aLIBRETURN 
aFile_Close(aIOLib ioRef,
	    aFileRef fileRef,
	    aErr* pErr);

aIO_EXPORT aLIBRETURN 
aFile_Read(aIOLib ioRef,
           aFileRef fileRef,
	   char* pBuffer,
	   const unsigned long nLength,
	   unsigned long* pActuallyRead,
	   aErr* pErr);

aIO_EXPORT aLIBRETURN 
aFile_Write(aIOLib ioRef,
	    aFileRef fileRef,
	    const char* pBuffer,
	    const unsigned long nLength,
  	    unsigned long* pActuallyWritten,
  	    aErr* pErr);

aIO_EXPORT aLIBRETURN 
aFile_Seek(aIOLib ioRef,
	   aFileRef fileRef,
	   const long nOffset,
	   aBool bFromStart,
	   aErr* pErr);

aIO_EXPORT aLIBRETURN 
aFile_GetSize(aIOLib ioRef,
	      aFileRef fileRef,
	      unsigned long* pulSize,
	      aErr* pErr);

aIO_EXPORT aLIBRETURN 
aFile_Delete(aIOLib ioRef,
	     const char *pFilename,
	     const aFileArea eArea,
	     aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Directory Routine definitions
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef aErr 
(*aDirectoryListProc)(const char* pFilename,
		      const unsigned long nSize,
		      void* ref);

aIO_EXPORT aLIBRETURN 
aDirectory_List(aIOLib ioRef,
  	        const aFileArea eArea,
		const char* pExtension,
		aDirectoryListProc listProc,
		void* vpRef,
		aErr* pErr);

#ifdef __cplusplus 
}
#endif



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * setting file routines
 */

typedef void* aSettingFileRef;

#ifdef __cplusplus
extern "C" {
#endif
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_Create(aIOLib ioRef,
		      const unsigned int nMaxSettingLen,
		      const char* pFileName,
		      aSettingFileRef* pSettingFileRef,
		      aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_GetInt(aIOLib ioRef,
		      aSettingFileRef settingFileRef,
		      const char* key, 
		      int* pInt,
		      const int nDefault,
		      aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_GetULong(aIOLib ioRef,
			aSettingFileRef settingFileRef,
			const char* key, 
			unsigned long* pULong,
			const unsigned long nDefault,
			aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_GetFloat(aIOLib ioRef,
			aSettingFileRef settingFileRef,
			const char* key, 
			float* pFloat,
			const float fDefault,
			aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_GetString(aIOLib ioRef,
			 aSettingFileRef settingFileRef,
			 const char* key, 
			 char** ppString,
			 const char* pDefault,
			 aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_GetInetAddr(aIOLib ioRef,
			   aSettingFileRef settingFileRef,
			   const char* key, 
			   unsigned long* pInetAddr,
			   const unsigned long pDefault,
			   aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_SetKey(aIOLib ioRef,
		      aSettingFileRef settingFileRef,
		      const char* pKey, 
		      const char* pValue,
		      aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_AddArguments(aIOLib ioRef,
			    aSettingFileRef settingFileRef,
			    const int argc, 
			    const char* argv[],
			    aErr* pErr);
  
  aIO_EXPORT aLIBRETURN
  aSettingFile_Destroy(aIOLib ioRef,
		       aSettingFileRef settingFileRef,
		       aErr* pErr);
  
#ifdef __cplusplus 
}
#endif



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Low level operation proc defs for I/O streams.  These need to 
 * be implimented for new stream types.
 */

typedef aErr 
(*aStreamGetProc)(char* pData, void* ref);

typedef aErr 
(*aStreamPutProc)(char* pData, void* ref);

typedef aErr 
(*aStreamDeleteProc)(void* ref);

typedef void* aStreamRef;

#define aStreamLibRef(aStreamRef) (*(aIOLib*)aStreamRef)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Stream operations.
 */

#ifdef __cplusplus
extern "C" {
#endif

aIO_EXPORT aLIBRETURN
aStream_CreateFromSettings(aIOLib ioRef, 
			   aSettingFileRef,
			   aStreamRef* pStreamRef,
			   aErr* pErr);

aIO_EXPORT aLIBRETURN
aStream_Create(aIOLib ioRef, 
	       aStreamGetProc getProc,
	       aStreamPutProc putProc,
	       aStreamDeleteProc deleteProc,
	       const void* procRef,
	       aStreamRef* pStreamRef,
	       aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_CreateFileInput(aIOLib ioRef, 
			const char *pFilename,
			const aFileArea eArea,
			aStreamRef* pStreamRef,
			aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_CreateFileOutput(aIOLib ioRef, 
			 const char *pFilename,
			 const aFileArea eArea,
			 aStreamRef* pStreamRef,
			 aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_CreateSerial(aIOLib ioRef, 
		     const char *pPortName, 
		     const unsigned int nBaudRate, 
		     aStreamRef* pStreamRef, 
		     aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_CreateSocket(aIOLib ioRef, 
		     const unsigned long address, 
		     const unsigned short port, 
		     const aBool bServer,
		     aStreamRef* pStreamRef, 
		     aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_CreateUSB(aIOLib ioRef, 
		  const unsigned int serialNum, 
		  aStreamRef* pStreamRef, 
		  aErr* pErr);
  
aIO_EXPORT aLIBRETURN 
aStreamBuffer_Create(aIOLib ioRef, 
		     const aMemSize nIncSize,
		     aStreamRef* pBufferStreamRef,
		     aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStreamBuffer_Get(aIOLib ioRef, 
		  aStreamRef bufferStreamRef,
		  aMemSize* aSize,
		  char** ppData,
		  aErr* pErr);

aIO_EXPORT aLIBRETURN
aStreamBuffer_Flush(aIOLib ioRef, 
		    aStreamRef bufferStreamRef,
		    aStreamRef flushStream,
		    aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_CreateZLibFilter(aIOLib ioRef, 
			 const aStreamRef streamToFilter,
			 const aFileMode eMode,
			 aStreamRef* pFilteredStreamRef,
			 aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_Read(aIOLib ioRef, 
	     aStreamRef streamRef,
	     char* pBuffer,
    	     const unsigned long nLength,
    	     aErr* pErr);

aIO_EXPORT aLIBRETURN
aStream_Write(aIOLib ioRef, 
	      aStreamRef streamRef,
	      const char* pBuffer,
    	      const unsigned long nLength,
    	      aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_ReadLine(aIOLib ioRef, 
		 aStreamRef streamRef,
		 char* pBuffer,
    		 const unsigned long nMaxLength,
    		 aErr* pErr);

aIO_EXPORT aLIBRETURN
aStream_WriteLine(aIOLib ioRef, 
		  aStreamRef streamRef,
		  const char* pBuffer,
		  aErr* pErr);

aIO_EXPORT aLIBRETURN
aStream_Flush(aIOLib ioRef, 
	      aStreamRef inStreamRef,
	      aStreamRef outStreamRef,
	      aErr* pErr);

aIO_EXPORT aLIBRETURN 
aStream_Destroy(aIOLib ioRef, 
		aStreamRef streamRef,
		aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Memory Pool allocator routines
 */

typedef void* aMemPoolRef;

#ifdef __cplusplus
extern "C" {
#endif

aIO_EXPORT aLIBRETURN 
aMemPool_Create(aIOLib ioRef,
		aMemSize objectSize,
		aMemSize blockSize,
		aMemPoolRef* pPoolRef,
		aErr* pErr);

aIO_EXPORT aLIBRETURN 
aMemPool_Alloc(aIOLib ioRef,
	       aMemPoolRef poolRef,
	       void** ppObj,
	       aErr* pErr);

aIO_EXPORT aLIBRETURN
aMemPool_Free(aIOLib ioRef,
	      aMemPoolRef poolRef,
	      void* pObj,
	      aErr* pErr);

aIO_EXPORT aLIBRETURN 
aMemPool_Destroy(aIOLib ioRef,
		 aMemPoolRef poolRef,
		 aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Symbol table routines
 */

typedef void* aSymbolTableRef;

#define aMAXIDENTIFIERLEN		32

typedef aErr (*symDataDeleteProc)(void* pData, void* ref);

#ifdef __cplusplus
extern "C" {
#endif


aIO_EXPORT aLIBRETURN 
aSymbolTable_Create(aIOLib ioRef,
		    aSymbolTableRef* pSymbolTableRef,
		    aErr* pErr);

aIO_EXPORT aLIBRETURN
aSymbolTable_Insert(aIOLib ioRef,
		    aSymbolTableRef symbolTableRef,
		    const char* identifier,
		    void* pData,
  		    symDataDeleteProc deleteProc,
  		    void* deleteRef,
		    aErr* pErr);

aIO_EXPORT aLIBRETURN
aSymbolTable_Find(aIOLib ioRef,
		  aSymbolTableRef symbolTableRef,
		  const char* identifier,
		  void** ppFoundData,
		  aErr* pErr);

aIO_EXPORT aLIBRETURN
aSymbolTable_Destroy(aIOLib ioRef,
		     aSymbolTableRef symbolTableRef,
		     aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * token types
 */

typedef unsigned char tkType;

#define tkInt		(tkType)0
#define tkFloat		(tkType)1
#define tkIdentifier	(tkType)2
#define tkSpecial	(tkType)3
#define tkString	(tkType)4
#define tkPreProc	(tkType)5
#define tkNewLine	(tkType)6

#define tkErrUntermCmnt		1
#define tkErrIncludeNotFnd	2
#define tkErrCompile		3
#define tkErrDuplicateDefine	4
#define tkErrBadArgumentList	5
#define tkErrBadFloat		6
#define tkErrMissingParen	7
#define tkErrInvalidChar        8
#define tkErrFilenameLength     9
#define tkErrXMLMissingClose	50

typedef struct aToken {
  tkType		eType;
  union {
    int			integer;
    float		floatVal;
    char		identifier[aMAXIDENTIFIERLEN];
    char*		string;
    char		special;
  } v;
} aToken;

typedef struct aTokenInfo {
  unsigned int		nLine;
  unsigned int		nColumn;
  char*			pSourceName;
  char*			pSourceLine;
  const char*           pTypeName;
} aTokenInfo;

typedef unsigned int tkError;

typedef aErr (*aTokenErrProc)(tkError error,
			      const unsigned int nLine,
			      const unsigned int nColumn,
			      const unsigned int nData,
			      const char* data[],
			      void* errProcRef);

typedef void* aTokenizerRef;

#ifdef __cplusplus
extern "C" {
#endif

aIO_EXPORT aLIBRETURN
aToken_GetInfo(aIOLib ioRef,
	       const aToken* pToken,
	       aTokenInfo* pTokenInfo,
	       aErr* pErr);

aIO_EXPORT aLIBRETURN
aTokenizer_Create(aIOLib ioRef,
		  aStreamRef tokenStream,
		  const char* streamName,
		  aFileArea eIncludeArea,
		  aTokenErrProc errProc,
		  void* errProcRef,
		  aTokenizerRef* pTokenizerRef,
		  aErr* pErr);

aIO_EXPORT aLIBRETURN
aTokenizer_Next(aIOLib ioRef,
		aTokenizerRef tokenizerRef,
		aToken** ppToken,
		aErr* pErr);

aIO_EXPORT aLIBRETURN
aTokenizer_PushBack(aIOLib ioRef,
		    aTokenizerRef tokenizerRef,
		    aToken* pToken,
		    aErr* pErr);

aIO_EXPORT aLIBRETURN
aTokenizer_Dispose(aIOLib ioRef,
		   aTokenizerRef tokenizerRef,
		   aToken* pToken,
		   aErr* pErr);

aIO_EXPORT aLIBRETURN
aTokenizer_Destroy(aIOLib ioRef,
		   aTokenizerRef tokenizerRef,
		   aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * XML data handling routines
 */

typedef void* aXMLRef;
typedef void* aXMLNodeRef;

typedef aErr (*aXMLHandleStart)(aXMLNodeRef node,
				const char* pKey,
				void* vpRef);

typedef aErr (*aXMLHandleContent)(aXMLNodeRef node,
				  const char* pKey,
				  const aToken* pToken,
				  void* vpRef);

typedef aErr (*aXMLHandleEnd)(aXMLNodeRef node,
			      aXMLNodeRef parent,
			      const char* pKey,
			      void* vpRef);

typedef struct aXMLCallbacks {
  aXMLHandleStart	handleStart;
  aXMLHandleContent	handleContent;
  aXMLHandleEnd		handleEnd;
  void*			ref;
} aXMLCallbacks;

#ifdef __cplusplus
extern "C" {
#endif

aIO_EXPORT aLIBRETURN
aXML_Create(aIOLib ioRef,
	    aStreamRef dataStream,
	    aTokenErrProc errProc,
	    void* errProcRef,
	    aXMLRef* pXML,
	    aErr* pErr);

aIO_EXPORT aLIBRETURN
aXML_Traverse(aIOLib ioRef,
	      aXMLRef XML,
	      const aXMLCallbacks* pCallbacks,
	      aErr* pErr);

aIO_EXPORT aLIBRETURN
aXML_Destroy(aIOLib ioRef,
	     aXMLRef XML,
	     aErr* pErr);

aIO_EXPORT aLIBRETURN
aXMLNode_FindKey(aIOLib ioRef,
		 aXMLNodeRef XMLNode,
		 const char* pKey,
		 aXMLHandleContent contentProc,
		 void* vpProcRef,
		 aErr* pErr);

aIO_EXPORT aLIBRETURN
aXMLNode_AddContent(aIOLib ioRef,
		    aXMLNodeRef XMLNode,
		    const char* pKey,
		    const char* pData,
		    aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Routines for managing an external shell program.
 */

typedef void* aShellRef;

typedef enum aShellDescriptor {
  eSTDOUT = 0,
  eSTDIN = 1,
  eSTDERR = 2
} aShellDescriptor;

#ifdef __cplusplus
extern "C" {
#endif
  
aIO_EXPORT aLIBRETURN
aShell_Create(aIOLib ioRef,
	      const char* pCommand,
	      aShellRef* pShellRef,
	      aErr* pErr);

aIO_EXPORT aLIBRETURN
aShell_GetStream(aIOLib ioRef,
		 aShellRef shellRef,
		 const aShellDescriptor descriptor,
		 aStreamRef* pStreamRef,
		 aErr* pErr);

aIO_EXPORT aLIBRETURN
aShell_Execute(aIOLib ioRef,
	       aShellRef shellRef,
	       aErr* pErr);

aIO_EXPORT aLIBRETURN
aShell_Completion(aIOLib ioRef,
	          aShellRef shellRef,
	          int* retValue,
	          aErr* pErr);

aIO_EXPORT aLIBRETURN
aShell_Destroy(aIOLib ioRef,
               aShellRef shellRef,
	       aErr* pErr);

#ifdef __cplusplus 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Common OS-independant I/O functions
 */

#ifdef __cplusplus
extern "C" {
#endif

aIO_EXPORT aLIBRETURN
aIO_GetMSTicks(aIOLib ioRef,
	       unsigned long* pNTicks,
	       aErr* pErr);

aIO_EXPORT aLIBRETURN
aIO_MSSleep(aIOLib ioRef,
	    const unsigned long msTime,
	    aErr* pErr);

aIO_EXPORT aLIBRETURN
aIO_GetInetAddr(aIOLib ioRef,
		unsigned long* pAddress,
		aErr* pErr);

#ifdef __cplusplus 
}
#endif



#endif /* _aIO_H_ */
