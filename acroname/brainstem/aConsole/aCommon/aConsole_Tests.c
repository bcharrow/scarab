/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsole_Tests.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent Console   */
/*		testing object.					   */
/*                                                                 */
/* Takes input file containing lines of:			   */
/*								   */
/*   <"filename"> <exitCode> <retVal> [<params>];  		   */
/*     or							   */
/*   target <module | host>;					   */
/*     or							   */
/*   mode <debug | load>;					   */
/*     or							   */
/*   concurrent <3>						   */
/*     or							   */
/*   input <packet | string>					   */
/*   :								   */
/*   :								   */
/*								   */
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

#ifdef aTESTS

#include "aCmd.tea"

#include "aConsoleText.h"
#include "aConsole_Tests.h"
#include "aModuleVM.h"
#include "aUtil.h"

/* testing modes */
typedef enum {
  kIdle,
  kLoad,
  kDebug
} aTestMode;

/* test types */
typedef enum {
  kVMCompile,
  kVMRun,
  kBatch
} aTestType;

/* input types */
typedef enum {
  kPacket,
  kString
} aTestInput;

/* code for a resource pool of indexes */
typedef void* aAvail;
static aErr sAvail_Create(const unsigned char size,
			  aAvail* pAvail);
static aErr sAvail_CheckOut(aAvail avail,
			    unsigned char* pVal);
static aErr sAvail_CheckIn(aAvail avail,
			   const unsigned char val);
static aErr sAvail_Destroy(aAvail avail);

/* one for each test currently running */
typedef struct aConsoleTest {
  aTestType		eType;
  char			srcName[aFILE_NAMEMAXCHARS];
  char			dsmName[aFILE_NAMEMAXCHARS];
  char			objName[aFILE_NAMEMAXCHARS];
  aVMExit		expectedExit;
  tSHORT		expectedReturn;
  char			expectedString[aMAXIDENTIFIERLEN];
  char			currentString[aMAXIDENTIFIERLEN];
  char			data[aSTEMMAXPACKETBYTES];
  unsigned char		dataSize;
  unsigned char		module;
  aTEAProcessID		pid;
  unsigned char		slot;
  aTestMode		eMode;
  aTestInput		eInput;
  aBool			bRequestedStep;
  aBool			bDone;
/*  unsigned int		nSessionID; */
  struct aConsoleTest*	pNext;
  struct aConsoleTests* pTests;
} aConsoleTest;

/* the overall test structure */
typedef struct aConsoleTests {
  aStreamRef		testStream;
  aTokenizerRef		tokenizer;
  unsigned char		nConcurrent;
  unsigned char		nSlots;
  unsigned char		nModule; /* 0 for host */
  aAvail		slots;
  aAvail		PIDs;
  int 			nTests;
  int			nSuccesses;
  int			nCurrent;
  aTestMode		eCurrentMode;
  aTestMode		eNextMode;
  aTestInput		eCurrentInput;
  aConsoleTest*		pNextBatch;
  aConsoleTest*		pTests;
  aBool			bDone;
  void*			vpConsole;
  aIOLib		ioRef;
} aConsoleTests;


#define aNUMCONCURRENTTESTS	1

typedef enum {
  kConcurrent,
  kTarget,
  kMode,
  kInput,
  kTest,
  kUnknown
} aTestCommand;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aConsoleTest* sConsole_NextTest(aConsoleTests* pTests);
static aErr sConsole_TestVMStart(aConsoleTest* pTest);
static aErr sConsole_TestBatchStart(aConsoleTest* pTest);
static aErr sConsole_TestDestroy(aConsole* pConsole,
				 aConsoleTest* pTest);
static aErr sConsole_TestComplete(aConsole* pConsole,
			   	  aConsoleTest* pTest, 
			   	  char* error);
static aErr sConsole_TestLoadLaunch(aConsole* pConsole,
				    const char* pFilename,
				    aConsoleTest* pTest,
				    aConsoleTests* pTests);
#ifdef aDEBUGGER
static aErr sConsole_TestDebugLaunch(aConsole* pConsole,
			      	     const char* pSourceName,
				     const char* pObjectName,
				     aConsoleTest* pTest);
#endif /* aDEBUGGER */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAvail_Create
 *
 * avail looks like:
 * byte 0 - number of resources
 * bytes 1 - n, resource booleans
 * byte n+1, last used resource
 */

aErr sAvail_Create(const unsigned char size,
		   aAvail* pAvail)
{
  unsigned char* p;

  aAssert(pAvail);

  p = (unsigned char*)aMemAlloc((aMemSize)(size + 2));

  if (p == NULL)
    return aErrMemory;

  aBZero(&p[1], size);

  /* set the size of the resource pool */
  p[0] = (unsigned char)size;

  /* set the last used so we get one first */
  p[size + 1] = 0;

  *pAvail = p;

  return aErrNone;

} /* sAvail_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAvail_CheckOut
 */

aErr sAvail_CheckOut(aAvail avail,
		     unsigned char* pVal)
{
  unsigned char i;
  unsigned char* p = (unsigned char*)avail;
  unsigned char previous = p[p[0] + 1];
  aBool bMustReuse = aFalse;
  aBool bFound = aFalse;
  unsigned char val = 0;

  aAssert(avail);

  /* look for a slot just past the last one used */
  for (i = (unsigned char)(previous + 2); 
       (bFound == aFalse) && (i <= p[0]); i++) {
    if (p[i] == 0) {
      if ((i - 1) == previous)
        bMustReuse = aTrue;
      else {
        val = (unsigned char)(i - 1);
        bFound = aTrue;
      }
    }
  }

  /* we may need to wrap back around */
  for (i = 1; (bFound == aFalse) && (i <= previous + 1); i++) {
    if (p[i] == 0) {
      if ((i - 1) == previous)
        bMustReuse = aTrue;
      else {
        val = (unsigned char)(i - 1);
        bFound = aTrue;
      }
    }
  }

  /* if we found one other than the last used, allocate it */
  if (bFound == aTrue) {
    p[val + 1] = 1;
    *pVal = val;
    return aErrNone;
  
  /* otherwise, we must reuse */
  } else if (bMustReuse == aTrue) {
    p[previous + 1] = 1;
    *pVal = previous;
    return aErrNone;
  }

  aAssert(*pVal < p[0]);

  return aErrNotFound;

} /* sAvail_CheckOut */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAvail_CheckIn
 */

aErr sAvail_CheckIn(aAvail avail,
		    const unsigned char val)
{
  unsigned char* p = (unsigned char*)avail;

  aAssert(p);
  aAssert(val <= p[0]);
  aAssert(p[val + 1] == 1);

  /* show that this was the last used */
  p[p[0] + 1] = val;

  /* make it free */
  p[val + 1] = 0;

  return aErrNone;

} /* sAvail_CheckIn */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAvail_Destroy
 */

aErr sAvail_Destroy(aAvail avail)
{
  aAssert(avail);

  aMemFree(avail);

  return aErrNone;

} /* sAvail_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_NextTest
 */

aConsoleTest* sConsole_NextTest(aConsoleTests* pTests)
{
  aConsoleTest* pTest = NULL;
  aToken* pToken = NULL;
  aTestCommand type = kUnknown;

  /* parse the test line and figure out what type of test it is */
  while ((pTest == NULL)
         && (pTests->bDone == aFalse)) {
    if (!aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
    			 &pToken, NULL)) {
      if (pToken->eType == tkIdentifier) {
        if (aStringCompare(pToken->v.identifier, "target") == 0) {
          aToken* pTargetToken;
          type = kTarget;
          if (!aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
          		       &pTargetToken, NULL)) {
            if ((pTargetToken->eType == tkIdentifier)
                && (aStringCompare(pTargetToken->v.identifier, "host") == 0)) {
              pTests->nModule = 0;
              aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		   aSTATUS_TEST_TARGETHOST, kDisplayStatus); 
            } else if ((pTargetToken->eType == tkInt)
                	&& (pTargetToken->v.integer < 256)
                	&& !(pTargetToken->v.integer % 2)) {
              char line[100];
              char num[10];
              pTests->nModule = (unsigned char)pTargetToken->v.integer;
              aStringFromInt(num, pTests->nModule);
              aStringCopySafe(line, 100, aSTATUS_TEST_TARGETMODULE);
              aStringCatSafe(line, 100, num);
              aConsole_DisplayLine((aConsole*)pTests->vpConsole, line, kDisplayStatus);
            } else {
              aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		   aSTATUS_TEST_TARGETBAD, kDisplayStatus);
            }
            aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
            		       pTargetToken, NULL);
          }
        } else if (!aStringCompare(pToken->v.identifier, "mode")) {
          aToken* pModeToken;
          type = kMode;
          if (!aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
          		       &pModeToken, NULL)) {
            if (pModeToken->eType == tkIdentifier) {
              if (!aStringCompare(pModeToken->v.identifier, "load")) {
                pTests->eNextMode = kLoad;
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_RUNMODE, 
				     kDisplayStatus);
#ifdef aDEBUGGER
              } else if (aStringCompare(pModeToken->v.identifier, "debug") == 0) {
                pTests->eNextMode = kDebug;
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_DEBUGMODE, kDisplayStatus);
#endif /* aDEBUGGER */
              } else
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_BADMODE, kDisplayStatus);
            }
            aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
            		       pModeToken, NULL);
          }
        } else if (!aStringCompare(pToken->v.identifier, "concurrent")) {
          aToken* pConcurrentToken;
          type = kConcurrent;
          if (!aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
          		       &pConcurrentToken, NULL)) {
            if ((pConcurrentToken->eType == tkInt) 
                && (pConcurrentToken->v.integer > 0)
		&& (pConcurrentToken->v.integer <= pTests->nSlots)) {
              char line[100];
              char num[10];
              if (pConcurrentToken->v.integer != pTests->nConcurrent) {
                pTests->nConcurrent = (unsigned char)pConcurrentToken->v.integer;
                if (pTests->PIDs)
      		  sAvail_Destroy(pTests->PIDs);
      	        sAvail_Create((unsigned char)pTests->nConcurrent,
			    &pTests->PIDs);
                aStringFromInt(num, pTests->nConcurrent);
                aStringCopySafe(line, 100, aSTATUS_TEST_CONCUR);
                aStringCatSafe(line, 100, num);
                aConsole_DisplayLine((aConsole*)pTests->vpConsole, 
				     line, kDisplayStatus);
	      }
            } else {
              aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		   aSTATUS_TEST_BADCONCUR,
				   kDisplayStatus);
            }
            aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
            		       pConcurrentToken, NULL);
          }
        } else if (!aStringCompare(pToken->v.identifier, "input")) {
          aToken* pInputToken;
          type = kInput;
          if (!aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
          		       &pInputToken, NULL)) {
            if (pInputToken->eType == tkIdentifier) {
              if (!aStringCompare(pInputToken->v.identifier, "packet")) {
                pTests->eCurrentInput = kPacket;
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_PACKETINPUT,
				     kDisplayStatus);
              } else if (aStringCompare(pInputToken->v.identifier, "string") == 0) {
                pTests->eCurrentInput = kString;
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_STRINGINPUT,
				     kDisplayStatus);
              } else
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_BADINPUT, kDisplayStatus);
            }
            aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
            		       pInputToken, NULL);
          }
        } else if (!aStringCompare(pToken->v.identifier, "slots")) {
          aToken* pTargetToken;
          type = kTarget;
          if (!aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
          		       &pTargetToken, NULL)) {
            if ((pTargetToken->eType == tkInt)
                && (pTargetToken->v.integer < 256)) {
              char line[100];
              char num[10];
              pTests->nSlots = (unsigned char)pTargetToken->v.integer;
              if (pTests->slots)
      		sAvail_Destroy(pTests->slots);
      	      sAvail_Create(pTests->nSlots, &pTests->slots);
              aStringFromInt(num, pTests->nSlots);
              aStringCopySafe(line, 100, aSTATUS_TEST_USINGSLOTS);
              aStringCatSafe(line, 100, num);
              aConsole_DisplayLine((aConsole*)pTests->vpConsole, 
				   line, kDisplayStatus);
            } else {
              aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		   aSTATUS_TEST_BADSLOT,
				   kDisplayStatus);
            }
            aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
            		       pTargetToken, NULL);
          }
        } else {
          aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                	       aSTATUS_TEST_BADINPUT, kDisplayStatus);
        }
        aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
        		   pToken, NULL);
 
      /* here we have an actual test, not directives */
      } else if (pToken->eType == tkString) {
        char extension[aFILE_NAMEMAXCHARS];
 	unsigned char maxLen = aSTEMMAXPACKETBYTES;

        type = kTest;
        pTest = (aConsoleTest*)aMemAlloc(sizeof(aConsoleTest));
        aAssert(pTest);
        aBZero(pTest, sizeof(aConsoleTest));
        pTest->pTests = pTests;

        /* figure out what type of test it is */
        aUtil_GetFileExtension(extension, pToken->v.string, NULL);
        if (aStringCompare(extension, ".tea") == 0) {
          pTest->eType = kVMCompile;
        } else if (aStringCompare(extension, ".cup") == 0) {
          pTest->eType = kVMRun;
        } else {
          aAssert(aStringCompare(extension, "") == 0);
          pTest->eType = kBatch;
        }

	/* save out the name of the test file */
        aStringCopySafe(pTest->srcName, aFILE_NAMEMAXCHARS, pToken->v.string);
        aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
        		   pToken, NULL);

        if (pTest->eType != kBatch) {
          aErr availErr;
          maxLen = 6; /* maximum input data length */
          pTest->module = pTests->nModule;
          pTest->eMode = pTests->eCurrentMode;
          pTest->eInput = pTests->eCurrentInput;
          availErr = sAvail_CheckOut(pTests->PIDs, &pTest->pid);
          aAssert(availErr == aErrNone);
          availErr = sAvail_CheckOut(pTests->slots, &pTest->slot);
          aAssert(availErr == aErrNone);

          /* scan for the exit code */
          if (!aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
          		       &pToken, NULL)) {
            if (pToken->eType == tkInt) {
              pTest->expectedExit = (aVMExit)pToken->v.integer;
            } else {
              type = kUnknown;
              aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		   aSTATUS_TEST_BADINPUT, kDisplayStatus);
            }
            aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
            		       pToken, NULL);
          }

          /* scan for the return value */
          if ((type != kUnknown)
              && !aTokenizer_Next(pTests->ioRef, pTests->tokenizer, 
              			  &pToken, NULL)) {
            if (pTest->eInput == kPacket) {
              if (pToken->eType == tkInt)
                pTest->expectedReturn = (tSHORT)pToken->v.integer;
              else
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_BADINPUT, kDisplayStatus);
            } else if (pTest->eInput == kString) {
              if (pToken->eType == tkString)
                aStringCopySafe(pTest->expectedString, aMAXIDENTIFIERLEN, pToken->v.string);
              else
                aConsole_DisplayLine((aConsole*)pTests->vpConsole,
                		     aSTATUS_TEST_BADINPUT, kDisplayStatus);
            }
            aTokenizer_Dispose(pTests->ioRef, pTests->tokenizer, 
            		       pToken, NULL);
          }
        }

        /* pick up any trailing data */
        aConsole_ParseRaw((aConsole*)pTests->vpConsole, 
        		  pTests->tokenizer,
        		  pTest->data,
        		  &pTest->dataSize,
        		  maxLen);
      }
    } else {
      pTests->bDone = aTrue;
    }
  }

  return pTest;

} /* sConsole_NextTest */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_TestLoadLaunch
 *
 * loads and launches a test either locally or over the wire
 */

aErr sConsole_TestLoadLaunch(
  aConsole* pConsole,
  const char* pFilename,
  aConsoleTest* pTest,
  aConsoleTests* pTests
)
{
  aErr consoleErr = aErrNone;

  /* now run it */
  if (consoleErr == aErrNone) {

    /* module 0 means local (host) tests */
    if (pTest->module == 0) {
      consoleErr = aConsole_Launch(pConsole,
			           pFilename,
			           aFileAreaTest,
			           pTest->data,
			           pTest->dataSize,
                                   aConsole_VMExit,
                                   (void *)pConsole,
                                   fTEAvmSetPID,
                                   &pTest->pid);

    /* any other module should be sent down the link */
    } else {
      aStreamRef fileStream;

      aStream_CreateFileInput(pConsole->ioLib,
      			      pFilename,
      			      aFileAreaTest,
      			      &fileStream,
      			      &consoleErr);
      if (consoleErr == aErrNone)
        consoleErr = aConsole_Load(pConsole,
      				   fileStream,
      				   pTest->module,
      				   pTest->slot);
      if (consoleErr == aErrNone)
        aStream_Destroy(pConsole->ioLib,
        		fileStream,
        		&consoleErr);

      if (consoleErr == aErrNone)
        consoleErr = aModuleVM_LaunchProcess(pConsole->stemLib,
					     pTest->module, 
					     pTest->slot,
					     pTest->data, 
					     pTest->dataSize,
					     bitVM_RUN_PID,
					     &pTest->pid);
    }
  }
  
  return consoleErr;

} /* sConsole_TestLoadLaunch */


#ifdef aDEBUGGER
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_TestDebugLaunch
 */

aErr sConsole_TestDebugLaunch(aConsole* pConsole,
			      const char* pSourceName,
			      const char* pObjectName,
			      aConsoleTest* pTest)
{
  aErr consoleErr = aErrNone;

  /* launch the program in the debug server 
   * this may blow away the session so make sure
   * no reference to the session follows this call 
   */
  if (consoleErr == aErrNone) {
    aTEAvmLaunchBlock lb;
    lb.data = pTest->data;
    lb.dataSize = pTest->dataSize;
    lb.exitProc = aConsole_VMExit;
    lb.exitRef = pConsole;
    lb.flags = fTEAvmSetPID;
    lb.pid = pTest->pid;

    {
      char line[100];
      aStringCopy(line, "launching debug ");
      aStringCat(line, pTest->srcName);
      aStem_DebugLine(pConsole->stemLib, line, NULL);
    }

    consoleErr = aTEADebugger_AddSession(pConsole->pDebugger,
    					 pTest->srcName,
    					 pTest->module,
    					 aFileAreaTest,
    					 aFileAreaTest,
    					 &lb);
  }
  
  return consoleErr;

} /* sConsole_TestDebugLaunch */
#endif /* aDEBUGGER */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_TestVMStart
 */

aErr sConsole_TestVMStart(aConsoleTest* pTest)
{
  aErr consoleErr = aErrNone;
  aConsoleTests* pTests = pTest->pTests;
  aConsole* pConsole = (aConsole*)pTests->vpConsole;

  aAssert(((pTest->eType == kVMCompile) 
           || (pTest->eType == kVMRun)));

  /* compile the code unless it is an object */
  if (consoleErr == aErrNone) {
    aString_GetFileRoot(pTest->objName, pTest->srcName);
    aStringCatSafe(pTest->objName, aFILE_NAMEMAXCHARS, ".cup");
    if (pTest->eType == kVMCompile)
      consoleErr = aConsole_Compile(pConsole, 
        			    fSteepGenerateCode, 
        			    NULL, NULL,
        			    pTest->srcName, 
        			    aFileAreaTest,
        			    pTest->objName,
        			    aFileAreaTest);
  }

  /* dissemble the code */
  if (consoleErr == aErrNone) {
    aString_GetFileRoot(pTest->dsmName, pTest->objName);
    aStringCatSafe(pTest->dsmName, aFILE_NAMEMAXCHARS, ".dsm");
    consoleErr = aConsole_Compile(pConsole, 
        			  fSteepDisassemble,
        			  NULL, NULL, 
        			  pTest->objName, 
        			  aFileAreaTest,
        			  pTest->dsmName,
        			  aFileAreaTest);
  }

  /* link in to the current tests */
  if (consoleErr == aErrNone) {
    pTests->nTests++;
    pTests->nCurrent++;
    pTest->pNext = pTests->pTests;
    pTests->pTests = pTest;
  } else {
    char line[100];
    aStringCopySafe(line, 100, aSTATUS_TEST_FAILED);
    aStringCatSafe(line, 100, pTest->srcName);
    aConsole_DisplayLine(pConsole, line, kDisplayStatus);
    aMemFree((aMemPtr)pTest);
    consoleErr = aErrNone;
  }

  if (consoleErr == aErrNone) {
    if (pTest->eMode == kLoad)
      consoleErr = sConsole_TestLoadLaunch(pConsole, 
      					   pTest->objName, 
    				     	   pTest, 
    				     	   pTests);
#ifdef aDEBUGGER
    else if (pTest->eMode == kDebug)
      consoleErr = sConsole_TestDebugLaunch(pConsole, 
      					    pTest->srcName, 
      					    pTest->objName, 
    				     	    pTest);
#endif /* aDEBUGGER */
  }

  return consoleErr;

} /* sConsole_TestVMStart */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_TestBatchStart
 */

aErr sConsole_TestBatchStart(aConsoleTest* pTest)
{
  aErr consoleErr = aErrNone;
  aConsoleTests* pTests = pTest->pTests;
  aConsole* pConsole = (aConsole*)pTests->vpConsole;
  aPacketRef packet;

  aVALIDCONSOLE(pConsole);

  /* run the batch */
  if (consoleErr == aErrNone) {
    consoleErr = aConsole_Batch(pConsole,
    				pTest->srcName,
    				aFileAreaTest,
    				aFalse);
    pTests->nTests++;
  }
  
  /* now, look for return data */
  if (consoleErr == aErrNone) {
    aErr packetErr;
   
    do {
      aStem_GetPacket(pConsole->stemLib, NULL, NULL, 5000,
      		      &packet, &packetErr);
    } while (packetErr == aErrNotFound);
    
    if (packetErr != aErrNone)
      consoleErr = packetErr;
  }
  
  /* compare the packet result */
  if (consoleErr == aErrNone) {
    char line[100];
    char data[aSTEMMAXPACKETBYTES];
    unsigned char address;
    unsigned char length;
    aPacket_GetData(pConsole->stemLib,
    		    packet,
    		    &address,
    		    &length,
    		    data,
    		    &consoleErr);
    
    /* we are done with the packet */
    if (consoleErr == aErrNone)
     aPacket_Destroy(pConsole->stemLib, packet, &consoleErr);

    /* check to see if it was the expected packet */
    if (consoleErr == aErrNone) {
      int i;
      aBool bSuccess = aTrue;

      /* glob up a status message */
      aStringCopySafe(line, 100, pTest->srcName);
      aStringCatSafe(line, 100, "..");
      
      /* make sure the returned data was the proper length */
      if (pTest->dataSize != length) {
        aStringCatSafe(line, 100, "failed, data len");
        bSuccess = aFalse;
      }

      /* compare the actual data */   
      for (i = 0; (i < length) && (bSuccess == aTrue); i++) {
        if (data[i] != pTest->data[i]) {
          char num[10];
          bSuccess = aFalse;
          aStringFromInt(num, i);
          aStringCatSafe(line, 100, "failed, byte ");
          aStringCatSafe(line, 100, num);
          aStringFromInt(num, data[i]);
          aStringCatSafe(line, 100, ", ");
          aStringCatSafe(line, 100, num);
          aStringCatSafe(line, 100, " != ");
          aStringFromInt(num, pTest->data[i]);
          aStringCatSafe(line, 100, num);
        }
      }
 
      /* record the success */
      if (bSuccess == aTrue) {
        pTests->nSuccesses++;
        aStringCatSafe(line, 100, "success");
      }

      /* display the actual output */
      consoleErr = aConsole_DisplayLine(pConsole, line, kDisplayStatus);
    }
  }
  
  /* either way, destroy the test */
  sConsole_TestDestroy(pConsole, pTest);

  return consoleErr;

} /* sConsole_TestBatchStart */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_TestDestroy
 */

aErr sConsole_TestDestroy(aConsole* pConsole,
			  aConsoleTest* pTest)
{
  aErr consoleErr = aErrNone;
  aConsoleTests* pTests = pTest->pTests;

  {
    char line[100];
    char num[10];
    aStringCopySafe(line, 100, "done with ");
    aStringCatSafe(line, 100, pTest->srcName);
    aStringCatSafe(line, 100, ", PID ");
    aStringFromInt(num, pTest->pid);
    aStringCatSafe(line, 100, num);
    aStem_DebugLine(pConsole->stemLib, line, NULL);
  }

#ifdef aDEBUGGER
  /* clean up any debug sessions */
  if ((consoleErr == aErrNone) 
      /* && pTest->pid */
      && (pTest->eMode == kDebug)) {
    consoleErr = aTEADebugger_Kill(pConsole->pDebugger,
      				   pTest->pid);
  }
#endif /* aDEBUGGER */

  /* unlink the test from the test list */
  if (consoleErr == aErrNone) {
    aErr availErr;
    aConsoleTest* pTemp = pTests->pTests;
    aConsoleTest* pPrev = NULL;
    while (pTemp && (pTemp != pTest)) {
      pPrev = pTemp;
      pTemp = pTemp->pNext;
    }
    if (pPrev != NULL)
      pPrev->pNext = pTest->pNext;
    else
      pTests->pTests = pTest->pNext;
      
    /* return the slot and process ID to the available slots */
    if (pTest->eType != kBatch) {
      availErr = sAvail_CheckIn(pTests->PIDs, pTest->pid);
      aAssert(availErr == aErrNone);
      availErr = sAvail_CheckIn(pTests->slots, pTest->slot);
      aAssert(availErr == aErrNone);
    }

    aMemFree((aMemPtr)pTest);
  }

  return consoleErr;

} /* sConsole_TestDestroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sConsole_TestComplete
 */

aErr sConsole_TestComplete(aConsole* pConsole,
			   aConsoleTest* pTest, 
			   char* error)
{
  aErr consoleErr = aErrNone;
  char line[100];
  aConsoleTests* pTests = (aConsoleTests*)pConsole->testData;

  /* leave some debug file traces */
  aStringCopySafe(line, 100, "debug exit ");
  aStringCatSafe(line, 100, pTest->srcName);
  aStem_DebugLine(pConsole->stemLib, line, NULL);

  if (pTest->eType == kVMCompile)
    aStringCopySafe(line, 100, pTest->srcName);
  else
    aStringCopySafe(line, 100, pTest->objName);
  aStringCatSafe(line, 100, "..");
  if (error[0] == 0) {
    aErr fileErr;
    aStringCatSafe(line, 100, "success");
    pTests->nSuccesses++;
    /* clean up the intermediate files */
    if (consoleErr == aErrNone) {
      aFile_Delete(pConsole->ioLib, pTest->dsmName, 
        	   aFileAreaTest, &fileErr);
    }
    if ((consoleErr == aErrNone)
        && (pTest->eType == kVMCompile)) {
      aFile_Delete(pConsole->ioLib, pTest->objName, 
        	   aFileAreaTest, &fileErr);
    }
  } else {
    aStringCatSafe(line, 100, error);
  }

  if (consoleErr == aErrNone)
    consoleErr = aConsole_DisplayLine(pConsole, line, kDisplayStatus);

  /* get rid of the actual test */
  if (consoleErr == aErrNone) {
    aAssert(pTests->nCurrent > 0);
    /* signal we are done waiting for the process */
    pTests->nCurrent--;
    consoleErr = sConsole_TestDestroy(pConsole, pTest);
  }

  return consoleErr;

} /* sConsole_TestComplete */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_CreateTests
 */

aErr aConsole_CreateTests(aConsole* pConsole,
			  aStreamRef testStream)
{ 
  aErr consoleErr = aErrNone;
  aConsoleTests* pTests;

  aVALIDCONSOLE(pConsole);

  /* first, create the testing block */
  if (consoleErr == aErrNone) {
    pTests = (aConsoleTests*)aMemAlloc(sizeof(aConsoleTests));
    if (pTests == NULL)
      consoleErr = aErrNone;
    else {
      aBZero(pTests, sizeof(aConsoleTests));
      pTests->testStream = testStream;
      pTests->nConcurrent = aNUMCONCURRENTTESTS;
      pTests->nSlots = aNSLOTDEFAULT;
      pTests->nModule = 0;
      pTests->eCurrentMode = kIdle;
      pTests->eNextMode = kLoad;
      pTests->eCurrentInput = kPacket;
      pTests->vpConsole = pConsole;
      pTests->ioRef = pConsole->ioLib;
      sAvail_Create(pTests->nSlots, &pTests->slots);
      sAvail_Create(pTests->nConcurrent, &pTests->PIDs);
    }
  }

  /* and then the tokenizer for the test file */
  if (consoleErr == aErrNone)
    aTokenizer_Create(pTests->ioRef, 
    		      pTests->testStream, 
    		      "tests", 
    		      aFileAreaTest, 
    		      NULL,
    		      NULL,
    		      &pTests->tokenizer, 
    		      &consoleErr);

  /* install this test object on the console */
  if (consoleErr == aErrNone) {
    aAssert(pConsole->testData == NULL);
    pConsole->testData = pTests;

    /* let the user know test are starting */
    consoleErr = aConsole_DisplayLine(pConsole, 
    				      aSTATUS_TEST_STARTING, 
    				      kDisplayStatus);
  }

  return consoleErr;

} /* aConsole_CreateTests */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_HandleTests
 */

aErr aConsole_HandleTests(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;

  aAssert(pConsole);

  if (pConsole->testData != NULL) {  
    aConsoleTests* pTests = (aConsoleTests*)pConsole->testData;

    if ((pTests->bDone == aTrue) 
        && (pTests->nCurrent == 0)) {
      char num[10];

      /* let the user know tests are finished */
      if (consoleErr == aErrNone)
        consoleErr = aConsole_DisplayLine(pConsole, 
    				          aSTATUS_TEST_FINISHED,
    				          kDisplayStatus);

      /* show some statistics */
      if (consoleErr == aErrNone) {
        aStringFromInt(num, pTests->nSuccesses);
        consoleErr = aConsole_BufferOutput(pConsole, num);
      }
      
      if (consoleErr == aErrNone)
        consoleErr = aConsole_BufferOutput(pConsole, " of ");

      if (consoleErr == aErrNone) {
        aStringFromInt(num, pTests->nTests);
        consoleErr = aConsole_BufferOutput(pConsole, num);
      }

      if (consoleErr == aErrNone)
        consoleErr = aConsole_BufferOutput(pConsole, " succeeded");
        
      if (consoleErr == aErrNone)
        consoleErr = aConsole_DisplayBuffer(pConsole, kDisplayStatus);

      /* actually destroy the tests */
      if (consoleErr == aErrNone) {
        aAssert(pTests->pTests == NULL);
        consoleErr = aConsole_DestroyTests(pConsole);
        pTests = NULL;
      }

    } /* if bDone = aTrue */

    /* start another test if not fully utilized */
    else if ((pTests->eNextMode == kIdle)
             && (pTests->nCurrent < pTests->nConcurrent)
             && (pTests->pNextBatch == NULL)
             && (pTests->bDone == aFalse)) {
      aConsoleTest* pTest = sConsole_NextTest(pTests);
      if (pTest != NULL) {
        switch (pTest->eType) {
        case kVMRun:
        case kVMCompile:
          consoleErr = sConsole_TestVMStart(pTest);
          break;
        case kBatch:
          pTests->pNextBatch = pTest;
          break;
        } /* switch */
      }

    /* if we are waiting on a batch and all other tests
     * are cleared out, run the batch test */    
    } else if ((pTests->pNextBatch != NULL)
             && (pTests->nCurrent == 0)) {
      consoleErr = sConsole_TestBatchStart(pTests->pNextBatch);
      pTests->pNextBatch = NULL;

    /* if we are switching modes and all previous mode tests
     * are done, switch modes */
    } else if ((pTests->eNextMode != kIdle)
               && (pTests->nCurrent == 0)) {
      pTests->eCurrentMode = pTests->eNextMode;
      pTests->eNextMode = kIdle;

    } 

#ifdef aDEBUGGER    
    /* now, let any debug tests step */
    else if ((pConsole->testData != NULL)
               && (consoleErr == aErrNone)) {
      aConsoleTest* pTest = pTests->pTests;
      while (pTest && (consoleErr == aErrNone)) {
        /* cache the pointer since stepping may blow away the 
         * current test on exit */
        aConsoleTest* pCur = pTest;
        pTest = pTest->pNext;

        /* do a debug step on debug vm processes */
        if (((pCur->eType == kVMCompile) ||
             (pCur->eType == kVMRun)) 
            && (pCur->eMode == kDebug))
          consoleErr = aTEADebugger_Step(pConsole->pDebugger,
  		                         pCur->pid);
      }
    }
#endif /* aDEBUGGER */

  } /* testData */

  return consoleErr;

} /* aConsole_HandleTests */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_DestroyTests
 */

aErr aConsole_DestroyTests(aConsole* pConsole) 
{
  aErr consoleErr = aErrNone;

  if (pConsole->testData != NULL) {
    aConsoleTest* pTest;
    aConsoleTests* pTests = (aConsoleTests*)pConsole->testData;

    /* clean up */
    pTest = pTests->pTests;
    while (pTests->pTests)
      sConsole_TestDestroy(pConsole, pTests->pTests);
    
    /* remove any current batch programs */
    if (pTests->pNextBatch != NULL) {
      aMemFree((aMemPtr)pTests->pNextBatch);
      pTests->pNextBatch = NULL;
    }

    if (pTests->tokenizer != NULL)
      aTokenizer_Destroy(pTests->ioRef, pTests->tokenizer, NULL);

    if (pTests->slots)
      sAvail_Destroy(pTests->slots);
    if (pTests->PIDs)
      sAvail_Destroy(pTests->PIDs);

    aMemFree((aMemPtr)pTests);

    pConsole->testData = NULL;
  }

  return consoleErr;

} /* aConsole_DestroyTests */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_CheckTestExit
 */

aBool aConsole_CheckTestExit(aConsole* pConsole,
			     unsigned char module,
		     	     aTEAProcessID pid,
		     	     aVMExit exitCode,
		     	     char* data,
		     	     unsigned char dataLen)
{
  aErr consoleErr = aErrNone;
  aBool bTestExited = aFalse;
  aConsoleTest* pTest = NULL;
  aConsoleTests* pTests = (aConsoleTests*)pConsole->testData;
  tSHORT retVal = 0;

  aAssert(pConsole);

  /* walk the tests to see if the dead vm is a test process */
  if (pTests != NULL) {
    pTest = pTests->pTests;
    while (pTest) {
      if ((pTest->module == module)
          && (pTest->pid == pid)) {
	/* collect the return value */
        switch (dataLen) {
        case 2:
          retVal = aUtil_RetrieveShort(data);
          break;
        case 1:
          retVal = (unsigned char)data[0];
          break;
        } /* switch */
        break;
      }
      pTest = pTest->pNext;
    }
  }

  if ((pTest != NULL) && (consoleErr == aErrNone)) {
    char line[100];
    char num[10];

    bTestExited = aTrue;
    
    if (pTest->eInput == kString) {

      /* glob up a status message */
      if (aStringCompare(pTest->currentString, 
      			 pTest->expectedString) == 0) {
      	line[0] = 0;
      } else {
        aStringCopySafe(line, 100, "failed, \"");
        aStringCatSafe(line, 100, pTest->currentString);
        aStringCatSafe(line, 100, "\"");
      }

    } else {

      /* glob up a status message */
      if (exitCode != pTest->expectedExit) {
        aStringCopySafe(line, 100, "failed exit = ");
        aStringFromInt(num, exitCode);
        aStringCatSafe(line, 100, num);
      } else if (retVal != pTest->expectedReturn) {
        aStringCopySafe(line, 100, "failed rv = ");
        aStringFromInt(num, retVal);
        aStringCatSafe(line, 100, num);
      } else {
        line[0] = 0;
      }
    }

    if (consoleErr == aErrNone)
      consoleErr = sConsole_TestComplete(pConsole, pTest, line);
  }
  
  return bTestExited;

} /* aConsole_CheckTestExit */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_CheckTestDisplay
 *
 * stepping may cause the test to go away
 */

aBool aConsole_CheckTestDisplay(aConsole* pConsole,
			    	unsigned char module,
		     	    	aTEAProcessID pid,
		     	    	char data)
{
  aErr consoleErr = aErrNone;
  aBool bTest = aFalse;
  aConsoleTest* pTest = NULL;
  aConsoleTests* pTests = (aConsoleTests*)pConsole->testData;

  aAssert(pConsole);

  /* walk the tests to see if the dead vm is a test process */
  if (pTests != NULL) {
    pTest = pTests->pTests;
    while (pTest) {
      if ((pTest->module == module)
          && (pTest->pid == pid)
          && (pTest->eInput == kString))
        break;
      pTest = pTest->pNext;
    }
  }

  if ((pTest != NULL) && (consoleErr == aErrNone)) {

    bTest = aTrue;

    /* linefeed signals the end of the input data so we 
     * can check the outcome */
    if (data == '\n') {

      pTest->bDone = aTrue;

    /* accumulate the new character */
    } else if (pTest->bDone == aFalse) {
      unsigned int len = (unsigned int)aStringLen(pTest->currentString);
      pTest->currentString[len++] = data;
      pTest->currentString[len] = 0;
    }
  }
  
  return bTest;

} /* aConsole_CheckTestDisplay */


#endif /* aTESTS */
