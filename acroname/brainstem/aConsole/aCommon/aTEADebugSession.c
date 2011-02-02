/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEADebugSession.c                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent TEA Debugger  */
/*              server.                                            */
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

#ifdef aDEBUGGER

#include "aTEA.h"
#include "aUtil.h"
#include "aTEADebugSession.h"
#include "aTEADebugInternal.h"
#include "aTEADebugExternal.h"
#include "aConsole.h"
#include "aStackOp.h"

/* image names are:
 * hxx - hilighted
 * nxx - normal
 * dxx - disabled */
#define opGoName	"go"
#define opStepInName	"si"
#define opStepOutName	"so"
#define opStepOverName	"sv"
#define opStopName	"st"
#define opKillName	"kl"
#define opSetBPName	"sb"
#define opClearBPName	"cb"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static aErr sHTMLEscape(char* result, 
		        const char* source);
static aErr sCreateHTMLDisplayFromFile(aIOLib ioRef,
				       const char* name,
				       const aFileArea area,
				       aTextDisplayRef* pDisplay,
				       unsigned int* pNLines);

static aErr sCreateInstructionArray(aTEADebugSession* pSession);

static void sFormatStackByte(char* source, aTEADebugSession* pSession, int i);
static void sAdvanceInstrIndex(aTEADebugSession* pSession);
static int sAddressToIndex(aTEADebugSession* pSession,
			   int nAddr,
			   int nInitIX,
			   aBool* pbExact);
static int sLineToIndex(aTEADebugSession* pSession,
			int nLine,
			int nInitIX,
			aBool* pbExact);

static aBool sCondition_EndGo(aTEADebugSession* pSession);
static aBool sCondition_EndStepOut(aTEADebugSession* pSession);
static aBool sCondition_EndStepOver(aTEADebugSession* pSession);
static aErr sOneStep(aTEADebugSession* pSession, aBool* pbExited);

static aErr sManageDebugStackOp(aTEADebugSession* pSession);
static aErr sRefreshCallStack(aTEADebugSession* pSession);
static aErr sRefreshRegisters(aTEADebugSession* pSession);
static aErr sRefreshCode(aTEADebugSession* pSession);
static aErr sRefreshStackBytes(aTEADebugSession* pSession,
			const tSTACK absStart,
			const int nBytes,
			const aBool bDirty);

static aBool sActiveButton(aTEADebugSession* pSession);

static aErr sPushStackFrame(aTEADebugSession* pSession,
			    const tADDRESS codePosition,
			    const char* pFrameName);
static aErr sPopStackFrame(aTEADebugSession* pSession);

static aErr sRenderDebugger(const unsigned int nParamIndex,
		            const unsigned int nBlockIndex,
		            aStreamRef reply,
		            void* vpRef);
static aErr sRenderProcList(const unsigned int nParamIndex,
		            const unsigned int nBlockIndex,
		            aStreamRef reply,
		            void* vpRef);
static aErr sRenderButtons(const unsigned int nParamIndex,
		           const unsigned int nBlockIndex,
		           aStreamRef reply,
		           void* vpRef);
static aErr sRenderLogo(const unsigned int nParamIndex,
		        const unsigned int nBlockIndex,
		        aStreamRef reply,
		        void* vpRef);
static aErr sRenderCodeLine(const unsigned int nParamIndex,
		            const unsigned int nBlockIndex,
		            aStreamRef reply,
		            void* vpRef);
static aErr sRenderRegisters(const unsigned int nParamIndex,
		         const unsigned int nBlockIndex,
		         aStreamRef reply,
		         void* vpRef);
static aErr sRenderStack(const unsigned int nParamIndex,
		         const unsigned int nBlockIndex,
		         aStreamRef reply,
		         void* vpRef);
static aErr sRenderVariable(const unsigned int nParamIndex,
		            const unsigned int nBlockIndex,
		            aStreamRef reply,
		            void* vpRef);
static aErr sRenderCallStack(const unsigned int nParamIndex,
		            const unsigned int nBlockIndex,
		            aStreamRef reply,
		            void* vpRef);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sProcessID
 */

char sProcessID(aTEADebugSession* pSession)
{
  return (char)(pSession->nSessionPID & 0x00FF);

} /* sProcessID */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHTMLEscape
 */

aErr sHTMLEscape(char* result, const char* source)
{
  const char* s = source;
  char* r = result;
  aBool bNonWhite = aFalse;
  int col = 0;

  while (*s) {

    switch (*s) {

    case '>':
      bNonWhite = aTrue;
      *r++ = '&';
      *r++ = 'g';
      *r++ = 't';
      *r++ = ';';
      break;

    case '<':
      bNonWhite = aTrue;
      *r++ = '&';
      *r++ = 'l';
      *r++ = 't';
      *r++ = ';';
      break;

    case '\t':
    case ' ':
      if (!(col & 1)) {
        *r++ = '&';
        *r++ = 'n';
        *r++ = 'b';
        *r++ = 's';
        *r++ = 'p';
        *r++ = ';';
      } else {
        *r++ = ' ';
      }
      break;
    
    default:
      bNonWhite = aTrue;
      *r++ = *s;
      break;

    } /* switch */

    col++;
    s++;

  } /* while *s */

  *r = 0;

  if (!bNonWhite)
    aStringCopy(r, "&nbsp;");

  return aErrNone;

} /* sHTMLEscape */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCreateHTMLDisplayFromFile
 */

aErr sCreateHTMLDisplayFromFile(aIOLib ioRef,
				const char* name,
				const aFileArea area,
				aTextDisplayRef* pDisplay,
				unsigned int* pNLines)
{
  aErr dsErr = aErrNone;
  aStreamRef file;
  char line[aTEADGBMAXDISPLAYWIDTH];
  char escaped[aTEADGBMAXDISPLAYWIDTH*3];
  char tmpName[aMAXIDENTIFIERLEN];
  aTextDisplayRef display = NULL;
  unsigned int nLines = 0;

  /* build a display buffer for the source file */  
  if (dsErr == aErrNone)
    dsErr = aTextDisplay_Create(1, 0, aTEADGBMAXDISPLAYWIDTH*3, 
    				&display);

  /* FORCE IT TO OPEN AN EXISTING DSM FILE */
  aString_GetFileRoot(tmpName, name);
  aStringCat(tmpName, ".dsm");

  if (dsErr == aErrNone)
    aStream_CreateFileInput(ioRef, tmpName, area, &file, &dsErr);

  while (dsErr == aErrNone) {
    aStream_ReadLine(ioRef, file, line,
      		     aTEADGBMAXDISPLAYWIDTH, &dsErr);
    if (dsErr == aErrNone) {
      sHTMLEscape(escaped, line);
      aTextDisplay_AddLine(display, escaped, 1);
      nLines++;
    }
  }

  /* eof is ok */
  if (dsErr == aErrEOF)
    dsErr = aErrNone;
    
  /* close the file since we are done with it */
  if (dsErr == aErrNone) {
    *pDisplay = display;
    *pNLines = nLines;
    aStream_Destroy(ioRef, file, &dsErr);
  } else {
    if (display)
      aTextDisplay_Destroy(display);
  }

  return dsErr;

} /* sCreateHTMLDisplayFromFile */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sCreateInstructionArray
 */

static aErr sCreateInstructionArray(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aFileRef file = NULL;
  aIOLib ioRef;
  char tmpName[aMAXIDENTIFIERLEN];
/*  tSTACK dataSizeFromFile;*/
  int nAddress;
  int nLen;
  int i;
  int n;

  aAssert(pSession);
  aAssert(pSession->pDebugger);

  ioRef = pSession->pDebugger->ioRef;

  /* figure out what file to open */
  aString_GetFileRoot(tmpName, pSession->pSourceName);
  aStringCat(tmpName, ".cup");

  /* open the bad boy */
  if (dsErr == aErrNone)
    aFile_Open(ioRef,
               tmpName,
               aFileModeReadOnly,
               pSession->eObjectArea,
               &file,
               &dsErr);

  /* get the file size */
  if (dsErr == aErrNone) {
    unsigned long size;
    aFile_GetSize(ioRef, 
    		  file, 
    		  &size, 
    		  &dsErr);
    pSession->codeSize = (tADDRESS)size;
  }


  /* allocate and read in the code */
  if (dsErr == aErrNone) {
    pSession->code = (char*)aMemAlloc((aMemSize)pSession->codeSize);
    if (pSession->code == NULL)
      dsErr = aErrMemory;
    else {
      aFile_Read(ioRef,
      		 file,
      		 pSession->code,
      		 pSession->codeSize,
      		 NULL,
      		 &dsErr);
    }
  }

  /* close the file no matter what as we are done with it */
  if (file)
    aFile_Close(ioRef, file, NULL);

  if ((dsErr == aErrNone) 
      && (pSession->codeSize > aTEA_HEADERRVPOS))
    pSession->retValSize = pSession->code[aTEA_HEADERRVPOS];

  /*  dataSizeFromFile = (tSTACK)tinybuff[5]; */
  /********************************************************/
  /* DATASIZEFROMFILE SHOULD MATCH LAUNCH BLOCK DATA SIZE */
  /********************************************************/


  if (dsErr == aErrNone) {
    
    /* parse stored code to determine instruction count */
    i = aTEA_HEADERSIZE;
    while (i < pSession->codeSize) {
      nLen = pSession->pDebugger->opLengths[(int)pSession->code[i]];
      i += (nLen + 1);
      pSession->nInstrCt++;
    }

    /* allocate and clear instruction array */
    pSession->pInstr = (aTEADebugInstr*)
      aMemAlloc(sizeof(aTEADebugInstr) * pSession->nInstrCt);
    aBZero(pSession->pInstr, pSession->nInstrCt * sizeof(aTEADebugInstr));


    /* second pass to fill instruction records */
    i = aTEA_HEADERSIZE;
    nAddress = 0;
    n = 0;
    while (i < pSession->codeSize) {
      nLen = pSession->pDebugger->opLengths[(int)pSession->code[i]] + 1;
      aMemCopy(pSession->pInstr[n].op, &pSession->code[i], 
      	      (aMemSize)nLen);
      pSession->pInstr[n].nAddress = nAddress;
      nAddress += nLen;
      i += nLen;
      n++;
    }  
  }
  
  return dsErr;

} /* sCreateInstructionArray */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * formatStackByte
 */

void sFormatStackByte(char* source, aTEADebugSession* pSession, int i)
{
  static char tohex[17] = "0123456789ABCDEF";
  char c = pSession->cStack[i];
  
  aStringCopy(source, "00  .");
  
  source[0] = tohex[(c & 0xF0) >> 4];
  source[1] = tohex[c & 0x0F];

  /* space through tilda are printable characters */
  source[4] = ((c < ' ') || (c > '~')) ? (char)'.' : c;
  
} /* formatStackByte */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sLineToIndex
 */

int sLineToIndex(aTEADebugSession* pSession,
		   int nLine,
		   int nInitIX,
		   aBool* pbExact)
{
  int pc;
  
  /* HACK FOR DSM */
  pc = pSession->nIndex - 4;
  
  return sAddressToIndex(pSession, pc, nInitIX, pbExact);
  
} /* sLineToIndex */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sAddressToIndex
 */

int sAddressToIndex(aTEADebugSession* pSession,
		    int nAddr,
		    int nInitIX,
		    aBool* pbExact)
{
  int offset;
  int nIX = nInitIX;
  int a1, a2, opbytes;
  
  /* calling routine passes an address */
  /* do heuristic-guided search for instruction with matching address */
  /* iterate on ((current index) - ((address offset) / 2)) */
  for (;;) {

    /* check for normal termination */
    opbytes = pSession->pDebugger->opLengths[
			    (int)pSession->pInstr[nIX].op[0]];
    a1 = pSession->pInstr[nIX].nAddress;
    a2 = pSession->pInstr[nIX].nAddress + opbytes;
    if ((a1 <= nAddr) && (nAddr <= a2)) break;
    
    /* check for abnormal termination (out of bounds) */
    if ((nAddr > a2) && (nIX == (pSession->nInstrCt - 1))) break;
    if ((nAddr < a1) && (nIX == 0)) break;

    /* iterate */
    offset = (nAddr - pSession->pInstr[nIX].nAddress);
    if ((offset / 2) != 0) offset /= 2;
    nIX += offset;
    if (nIX < 0) nIX = 0;
    if (nIX > (pSession->nInstrCt - 1)) nIX = (pSession->nInstrCt - 1);
  }
  
  /* this is true if address lines up with start of instruction */
  *pbExact = (nAddr == a1);

  return nIX;
  
} /* sAddressToIndex */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * advanceInstrIndex(aTEADebugSession* pSession)
 */

void sAdvanceInstrIndex(aTEADebugSession* pSession)
{
  int nIX = pSession->nInstrIndex;
  int nPC = pSession->SB.pc;
  aBool bDummy;

  pSession->nInstrIndex = sAddressToIndex(pSession, nPC, nIX, &bDummy);
  
} /* advanceInstrIndex */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRefreshStackBytes
 */

aErr sRefreshStackBytes(aTEADebugSession* pSession,
			const tSTACK absStart,
			const int nBytes,
			const aBool bDirty)
{
  aErr dsErr = aErrNone;
  char stackbuff[8];
  tSTACK soffset = absStart;
  unsigned char nTotal;
  unsigned char n;
  tSTACK i, j;
  
  nTotal = (unsigned char)nBytes;

  while ((nTotal > 0) 
         && (dsErr == aErrNone) 
         && (soffset < aDBG_STACK_SIZE)) {
    n = (nTotal > 6) ? (unsigned char)6 : nTotal;
    dsErr = pSession->viewstack(pSession, 1, soffset, n, stackbuff);

    /* this loop copies all retrieved bytes to debug stack */
    /* it flags changed bytes within stack and new bytes on top */
    /* (any byte at or beyond old SP is considered changed) */
    for (i = soffset, j = 0; i < soffset + n; i++, j++) {
      if ((pSession->cStack[i] != stackbuff[j])
          || (i >= pSession->sp)
          || bDirty)
        pSession->fStackChg[i] = aTrue;
      pSession->cStack[i] = stackbuff[j];
    }
    
    soffset += n;
    nTotal -= n;
  }

  /* stack frame will need refresh */
  pSession->nDirtyFrames |= aDBG_DIRTY_STACK;

  return dsErr;
  
} /* sRefreshStackBytes */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRefreshCallStack
 */

aErr sRefreshCallStack(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  /* call stack frame will need refresh */
  pSession->nDirtyFrames |= aDBG_DIRTY_CALLS;
  
  return dsErr;
  
} /* sRefreshCallStack */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRefreshRegisters
 */

aErr sRefreshRegisters(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  
  pSession->nRegInfo[0] = pSession->cp;
  pSession->nRegInfo[1] = pSession->sp;
  pSession->nRegInfo[2] = (pSession->statusReg & 0x02) ? 1 : 0;
  pSession->nRegInfo[3] = (pSession->statusReg & 0x08) ? 1 : 0;
  pSession->nRegInfo[4] = (pSession->statusReg & 0x04) ? 1 : 0;
  pSession->nRegInfo[5] = (pSession->statusReg & 0x10) ? 1 : 0;

  /* register frame will need refresh */
  pSession->nDirtyFrames |= aDBG_DIRTY_REGS;
  
  return dsErr;
  
} /* sRefreshRegisters */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRefreshCode
 */

aErr sRefreshCode(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aBool bExited;
  
  aAssert(pSession);
  
  bExited = (pSession->processState & fPStateExited);
  
  /* HACKS FOR THE CURRENT DSM FORMAT */
  if (!bExited) {
    pSession->nCurrentLine = 5 + (unsigned int)pSession->cp;
  } else {
    pSession->nCurrentLine = 5 + (unsigned int)pSession->pInstr[pSession->nInstrIndex].nAddress;
  }

  /* code frame will need refresh */
  pSession->nDirtyFrames |= aDBG_DIRTY_CODE;
  
  return dsErr;
  
} /* sRefreshCode */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sManageDebugStack
 */

aErr sManageDebugStackOp(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  aTEADebugStackOp* q;
  aTEAvmStepBlock* pSB;
  unsigned char m;
  unsigned char n;
  char* oldstack;
  aBool bChange = aTrue;
  tADDRESS oldsp;
  tADDRESS newsp;
  tSTACK soffset;
  tSTACK absStart;
  int nNew;

  
  pSB = &pSession->SB;
  q = &aTEADebugStackOpArray[(int)pSB->op[0]];
  oldstack = (char*)&pSession->cStack;
  oldsp = pSession->sp;
  newsp = pSB->sp;


  /* wipe out old change flags */
  aBZero(pSession->fStackChg, sizeof(aBool) * aDBG_STACK_SIZE);
  
  
  if (q->cAuxChgType == aSZDEP_ZERO) {
  
    switch (q->cUpdateType)
    {
      case aCHGTO_NONE:
        bChange = aFalse;
        break;

      case aCHGTO_TOPN:
        m = aSTACKMODE_REL;
        soffset = q->cRefreshCt;
        break;

      case aCHGTO_RELS:
        /* offset from instruction */
        m = aSTACKMODE_REL;
        soffset = (tSTACK)pSB->op[1];
        break;

      case aCHGTO_RELX:
        /* offset comes from top of "old" stack */
        m = aSTACKMODE_REL;
        soffset = (tSTACK)oldstack[oldsp - 1];
        break;

      case aCHGTO_ABSS:
        /* address from instruction */
        m = aSTACKMODE_ABS;
        soffset = (tSTACK)aTEA_RetrieveAddress(&pSB->op[1]);
        break;

      case aCHGTO_ABSX:
        /* address comes from top of "old" stack */
        m = aSTACKMODE_ABS;
        soffset = (tSTACK)aTEA_RetrieveAddress(&oldstack[oldsp - 2]);
        break;

      default:
        bChange = aFalse;
        break;
    }


    if (bChange) {

      n = (unsigned char)q->cRefreshCt;

      /* set up index for copying retrieved stack data */
      switch (m)
      {
        case 0:  /* relative */
          absStart = (tSTACK)(pSB->sp - soffset);
          break;
        case 1:  /* absolute */
          absStart = soffset;
          break;
        default: /* catch-all (no data copying) */
          n = 0;
          break;
      }
       
      dsErr = sRefreshStackBytes(pSession, absStart, n, aTrue);
    }
    
  } else {
    
    /* DATA DEPENDENT CASES DEPEND ON NEW STACK SIZE */

    absStart = (tSTACK)(oldsp + q->cKnownSizeChg);
    nNew = newsp - absStart;

    /* a positive value indicates a push and requires refresh */    
    if (nNew > 0)
      dsErr = sRefreshStackBytes(pSession, absStart, nNew, aTrue);
  }
  
  return dsErr;

} /* sManageDebugStack */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aBool sCondition_EndGo(aTEADebugSession* pSession)
{
  /* go ends on breakpoint */
  return
  (pSession->pInstr[pSession->nInstrIndex].cFlags & aDBG_MASK_BREAKPT);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aBool sCondition_EndStepOut(aTEADebugSession* pSession)
{
  /* step out ends on breakpoint or if we just executed return */
  return
  ((pSession->SB.op[0] == op_RETURN) ||
  (pSession->pInstr[pSession->nInstrIndex].cFlags & aDBG_MASK_BREAKPT));
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aBool sCondition_EndStepOver(aTEADebugSession* pSession)
{
  /* step over ends on breakpoint or if we just popped out */
  return
  ((pSession->nInstrIndex == pSession->nStepOverTarg) ||
  (pSession->pInstr[pSession->nInstrIndex].cFlags & aDBG_MASK_BREAKPT));
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sOneStep
 */

aErr sOneStep(aTEADebugSession* pSession, aBool* pbExited)
{
  aErr dsErr = aErrNone;
  aBool bExited;

  aAssert(pSession);
  aAssert(pSession->step);
  
  /* exit checks are done prior to this level */
  aAssert(!(pSession->processState & fPStateExited));
  
  pSession->SB.pid = (unsigned char)(pSession->nSessionPID & 0x00FF);
  pSession->SB.op = pSession->pInstr[pSession->nInstrIndex].op;
  dsErr = pSession->step(pSession);

  pSession->processState = pSession->SB.processState;
  bExited = (pSession->processState & fPStateExited);
  
  if ((dsErr == aErrNone) && !bExited) {

    /* manage our call stack */
    if (pSession->SB.op[0] == op_CALL) {
      
      /* this call below will eventually fill in the stack with
       * routine names when they are available in the records */
      dsErr = sPushStackFrame(pSession, 
			aTEA_RetrieveAddress(&pSession->SB.op[1]),
			NULL);
    } else if (pSession->SB.op[0] == op_RETURN) {
      dsErr = sPopStackFrame(pSession);
    }

    sAdvanceInstrIndex(pSession);
  }
  
  (*pbExited) = bExited;
  
  return dsErr;

} /* sOneStep */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sActiveButton
 */

static aBool sActiveButton(aTEADebugSession* pSession)
{
  aBool flag = aFalse;
  aBool bExited = (pSession->processState & fPStateExited);
  aBool bAboutToStop = (pSession->bRunning && pSession->bStopNow);
  aBool bStopped = (!pSession->bRunning && !pSession->bStopNow);

  switch (pSession->nIndex)
  {
  case 0: /* go */
  case 1: /* stepin */
  case 2: /* stepout */
  case 3: /* stepover */
    if ((bStopped || bAboutToStop) && !bExited) flag = aTrue;
    break;
  case 4: /* stop */
    if (!(bStopped || bAboutToStop) && !bExited) flag = aTrue;
    break;
  case 5: /* kill */
    flag = aTrue;
    break;
  }
  
  return flag;

} /* sActiveButton */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPushStackFrame
 */

aErr sPushStackFrame(aTEADebugSession* pSession,
		     const tADDRESS codePosition,
		     const char* pFrameName)
{
  aErr dsErr = aErrNone;
  aTEADebugStackFrame* pFrame;

  aAssert(pSession);

  pFrame = (aTEADebugStackFrame*)aMemAlloc(sizeof(aTEADebugStackFrame));
  if (pFrame) {
    aBZero(pFrame, sizeof(aTEADebugStackFrame));
    pFrame->address = codePosition;
    pFrame->pName = pFrameName;
    pFrame->pNext = pSession->pStackFrames;
    pSession->pStackFrames = pFrame;
  } else {
    dsErr = aErrMemory;
  }
  
  /* this routine may also push all the variables for this 
   * stack frame when available */

  pSession->nDirtyFrames |= aDBG_DIRTY_CALLS;

  return dsErr;

} /* sPushStackFrame */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sPopStackFrame
 */

aErr sPopStackFrame(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  aTEADebugStackFrame* pOld;
  pOld = pSession->pStackFrames;
  aAssert(pOld);
  pSession->pStackFrames = pOld->pNext;
  aMemFree(pOld);

  /* this routine will strip the old variables and replaces
   * them with the caller's so it will dirty the variable pane */

  pSession->nDirtyFrames |= aDBG_DIRTY_CALLS | aDBG_DIRTY_VARS;

  return dsErr;

} /* sPopStackFrame */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_TimeSlice
 */

aErr aTEADebugSession_TimeSlice(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aBool bExited = aFalse;
  aBool bStopped = aFalse;

  aAssert(pSession);
  aAssert(pSession->step);
  aAssert(pSession->endcheck);

  bExited = (pSession->processState & fPStateExited);
  
  if (!bExited) {

    dsErr = sOneStep(pSession, &bExited);  
    bStopped = (pSession->endcheck(pSession)
                || bExited
                || pSession->bStopNow);
  }

  if (bStopped) {

    /* indicate that process has stopped */
    pSession->bRunning = aFalse;
    pSession->bStopNow = aFalse;

    /* refresh registers after stopping */
    pSession->cp = pSession->SB.pc;
    pSession->sp = pSession->SB.sp;
    pSession->statusReg = pSession->SB.processState;
    
    /* wipe out old change flags */
    aBZero(pSession->fStackChg, sizeof(aBool) * aDBG_STACK_SIZE);
  
    /* refresh everything */
    sRefreshRegisters(pSession);
    sRefreshCallStack(pSession);
    sRefreshStackBytes(pSession, 0, pSession->sp, aFalse);
    sRefreshCode(pSession);
  }

  return dsErr;
  
} /* aTEADebugSession_TimeSlice */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_Create
 */

aErr aTEADebugSession_Create(const aTEADbgCreateBlock* pCB,
			     aTEADebugSession** ppSession)
{
  aErr dsErr = aErrNone;
  aTEADebugSession* pSession = NULL;
  int i;


  aAssert(pCB);
  aAssert(pCB->pDebugger);
  
  if (dsErr == aErrNone) {
    pSession = (aTEADebugSession*)
    			aMemAlloc(sizeof(aTEADebugSession));
    if (pSession) {
      aBZero(pSession, sizeof(aTEADebugSession));

      if (dsErr == aErrNone) {
      
        pSession->bKillNow = aFalse;
        pSession->pDebugger = pCB->pDebugger;
        aStringCopy(pSession->pSourceName, pCB->pFilename);
        pSession->eSourceArea = pCB->eSourceFileArea;
        pSession->eObjectArea = pCB->eObjectFileArea;
        pSession->nCurrentLine = 1;
        pSession->nWindowLines = 23; /* hard-coded default */
        pSession->module = pCB->module;
        pCB->pLB->flags |= fTEAvmDebug;

        if (pSession->module == 0) {

          /* INTERNAL DEBUGGING WITHIN TEA VM */
          pSession->launch = aTEADebugInternal_Launch;
          pSession->step = aTEADebugInternal_Step;
          pSession->kill = aTEADebugInternal_Kill;
          pSession->viewstack = aTEADebugInternal_ViewStack;

        } else {

          /* EXTERNAL DEBUGGING IN PHYSICAL MODULE */
          pSession->launch = aTEADebugExternal_Launch;
          pSession->step = aTEADebugExternal_Step;
          pSession->kill = aTEADebugExternal_Kill;
          pSession->viewstack = aTEADebugExternal_ViewStack;
        }
      }
      
      if (dsErr == aErrNone)
        dsErr = sCreateInstructionArray(pSession);

      /* set the code in the launch block for use in launching */
      pCB->pLB->code = pSession->code;
      pCB->pLB->codeSize = pSession->codeSize;
      pCB->pLB->retValSize = pSession->retValSize;

      /* initialize call stack frames with one entry */
      if (dsErr == aErrNone) {
        static char basename[] = "global";
        dsErr = sPushStackFrame(pSession, 0, basename);
      }

/**** DEBUG ****/
#if 0
{
 if (1) pSession->pInstr[5].cFlags |= aDBG_MASK_BREAKPT;
 if (0) {
  char i;
  aTEADebugCSNode** p;
  p = &pSession->pCSNode;
  for (i=0; i<1; i++) {
   (*p) = aMemAlloc(sizeof(aTEADebugCSNode));
   (*p)->cp = i;
   (*p)->pNext = NULL;
   p = &((*p)->pNext);
   pSession->nCSArraySize++;
  }
  sRefreshCallStack(pSession);
 }
}
#endif

      if (dsErr == aErrNone) {  
        /* mirror stack start state (input and return bytes) */
        pSession->sp = pSession->retValSize;
        for (i = 0; i < pCB->pLB->dataSize; i++)
          pSession->cStack[pSession->sp++] = pCB->pLB->data[i];
        for (i=0; i < pSession->sp; i++)
          pSession->fStackChg[i] = aTrue;
        /* advance past header -- WILL VARY WITH DSM FORMAT */
        pSession->nCurrentLine = 5;

        /* will need to render everything at start */
        pSession->nDirtyFrames = aDBG_DIRTY_ALL;
      }
      
      if (dsErr == aErrNone) {
        dsErr = pSession->launch(pSession, pCB->pLB);
        if (dsErr == aErrNone)
          /* high byte is module, low byte is process ID */
          pSession->nSessionPID = pCB->pLB->pid;
          pSession->nSessionPID |= ((unsigned int)pSession->module << 8);
      }
      
    } else {
      dsErr = aErrMemory;
    }
  }
  
  /* create a line buffer to remove any fixed length restriction
   * for code lines */
  if (dsErr == aErrNone)
    aStreamBuffer_Create(pCB->pDebugger->ioRef,
                         40, &pSession->lineBuffer, &dsErr);

  if (dsErr == aErrNone)
    *ppSession = pSession;

  return dsErr;

} /* aTEADebugSession_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_Destroy
 */

aErr aTEADebugSession_Destroy(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  if (pSession) {

    /* pop all the stack frames off */
    while (pSession->pStackFrames)
      sPopStackFrame(pSession);

    if (pSession->code)
      aMemFree(pSession->code);

    if (pSession->pInstr)
      aMemFree(pSession->pInstr);

    if (pSession->sourceDisplayRef)
      aTextDisplay_Destroy(pSession->sourceDisplayRef);

    if (pSession->lineBuffer)
      aStream_Destroy(aStreamLibRef(pSession->lineBuffer),
      		      pSession->lineBuffer, NULL);

    aMemFree(pSession);

  } else 
    dsErr = aErrNotFound;

  return dsErr;

} /* aTEADebugSession_Destroy */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_StepIn
 */

aErr aTEADebugSession_StepIn(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aBool bExited;

  aAssert(pSession);
  aAssert(pSession->step);

  bExited = (pSession->processState & fPStateExited);
  
  if (!bExited) {

    dsErr = sOneStep(pSession, &bExited);
    
    /* error may cause premature exit */
    
    if ((dsErr == aErrNone) && !bExited) {        
      sManageDebugStackOp(pSession);
      pSession->cp = pSession->SB.pc;
      pSession->sp = pSession->SB.sp;
      pSession->statusReg = pSession->SB.stateReg;
      sRefreshRegisters(pSession);
      sRefreshCode(pSession);
    }
  }
  
  return dsErr;

} /* aTEADebugSession_StepIn */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_StepOut
 */

aErr aTEADebugSession_StepOut(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  aAssert(pSession);

  pSession->bRunning = aTrue;  
  pSession->endcheck = sCondition_EndStepOut;
  
  /* force the logo to redraw when we run to handle stop checks */
  /* force redraw of windows to grey them while running */
  pSession->nDirtyFrames |= (aDBG_DIRTY_ALL & (~aDBG_DIRTY_BUTTONS));
  
  return dsErr;

} /* aTEADebugSession_StepOut */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_StepOut
 */

aErr aTEADebugSession_StepOver(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  aAssert(pSession);

  pSession->nStepOverTarg = pSession->nInstrIndex + 1;
  pSession->bRunning = aTrue;  
  pSession->endcheck = sCondition_EndStepOver;

  /* force the logo to redraw when we run to handle stop checks */
  /* force redraw of windows to grey them while running */
  pSession->nDirtyFrames |= (aDBG_DIRTY_ALL & (~aDBG_DIRTY_BUTTONS));
  
  return dsErr;

} /* aTEADebugSession_StepOver */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_Go
 */

aErr aTEADebugSession_Go(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  aAssert(pSession);

  pSession->bRunning = aTrue;  
  pSession->endcheck = sCondition_EndGo;

  /* force the logo to redraw when we run to handle stop checks */
  /* force redraw of windows to grey them while running */
  pSession->nDirtyFrames |= (aDBG_DIRTY_ALL & (~aDBG_DIRTY_BUTTONS));
  
  return dsErr;

} /* aTEADebugSession_Go */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_Stop
 */

aErr aTEADebugSession_Stop(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;

  aAssert(pSession);

  /* can only stop a process if it's running */
  if (pSession->bRunning)
    pSession->bStopNow = aTrue;  

  /* after a stop, all frames need refresh */
  pSession->nDirtyFrames = aDBG_DIRTY_ALL;

  return dsErr;

} /* aTEADebugSession_Stop */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_Kill
 */

aErr aTEADebugSession_Kill(aTEADebugSession* pSession)
{
  aErr dsErr = aErrNone;
  aAssert(pSession);

  /* force end to process time-slicing */
  /* kill VM process if it hasn't exited already */
  /* mark view for death */
  pSession->bRunning = aFalse;
  if (!(pSession->processState & fPStateExited))
    pSession->kill(pSession);
  pSession->bPrepareToKill = aTrue;

  return dsErr;

} /* aTEADebugSession_Kill */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_ProcessParams
 */

aErr aTEADebugSession_ProcessParams(aTEADebugSession* pSession,
			       	    aSymbolTableRef params)
{
  aErr dsErr = aErrNone;
  aErr symErr;
  char* pValue;
  int i;

  /* check for an operation */
  if ((dsErr == aErrNone)
      && !aSymbolTable_Find(pSession->pDebugger->ioRef,
    		            params, "op", (void**)&pValue, 
    		            &symErr)) {

    if (!aStringCompare(pValue, opGoName)) {
      dsErr = aTEADebugSession_Go(pSession);
    } else if (!aStringCompare(pValue, opStepInName)) {
      dsErr = aTEADebugSession_StepIn(pSession);
    } else if (!aStringCompare(pValue, opStepOutName)) {
      dsErr = aTEADebugSession_StepOut(pSession);
    } else if (!aStringCompare(pValue, opStepOverName)) {
      dsErr = aTEADebugSession_StepOver(pSession);
    } else if (!aStringCompare(pValue, opStopName)) {
      dsErr = aTEADebugSession_Stop(pSession);
    } else if (!aStringCompare(pValue, opKillName)) {
      dsErr = aTEADebugSession_Kill(pSession);
    } else if (!aStringCompare(pValue, opSetBPName)) {
      /* go get parameter "i" into index variable */
      dsErr = aSymbolTable_GetInt(pSession->pDebugger->ioRef, 
      			   params, "i", &i);
      if ((dsErr == aErrNone) && (i < pSession->nInstrCt)) {
        aAssert(!(pSession->pInstr[i].cFlags & aDBG_MASK_BREAKPT));
        pSession->pInstr[i].cFlags |= aDBG_MASK_BREAKPT;
      }
    } else if (!aStringCompare(pValue, opClearBPName)) {
      /* go get parameter "i" into index variable */
      dsErr = aSymbolTable_GetInt(pSession->pDebugger->ioRef, 
      			   params, "i", &i);
      if ((dsErr == aErrNone) && (i < pSession->nInstrCt)) {
        aAssert((pSession->pInstr[i].cFlags & aDBG_MASK_BREAKPT));
        pSession->pInstr[i].cFlags &= ~aDBG_MASK_BREAKPT;
      }
    }
  }
  
  return dsErr;

} /* aTEADebugSession_ProcessParams */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderCodeLine
 */

aErr sRenderCodeLine(const unsigned int nParamIndex,
		     const unsigned int nBlockIndex,
		     aStreamRef reply,
		     void* vpRef)
{
  aErr dsErr = aErrNone;
  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;

  /* default */
  char breakpoint[100];
  char num[30];
  aStringCopy(breakpoint, "&nbsp;");


  /* stop when pre-determined end line reached */   
  
  if (pSession->nIndex < (int)pSession->nCodeRenderEnd) {
    switch (nParamIndex) {

    /* parameter ^0 is the breakpoint cell content */
    case 0:
      {
        int nIX;
        aBool bExact;
        nIX = sLineToIndex(pSession, pSession->nIndex, 0, &bExact);
        if (bExact) {
          aBool bpSet = 
          	pSession->pInstr[nIX].cFlags & aDBG_MASK_BREAKPT;
          aStringCopy(breakpoint, "<IMG SRC='/aAsset/");
          if (bpSet) {
            aStringCat(breakpoint, "bp.jpg");
          } else {
            aStringCat(breakpoint, "nbp.jpg");
          }
          aStringCat(breakpoint, "' onclick='");
          if (bpSet) 
            aStringCat(breakpoint, opClearBPName"(");
          else
            aStringCat(breakpoint, opSetBPName"(");
          aStringFromInt(num, nIX);
          aStringCat(breakpoint, num);
          aStringCat(breakpoint, ",");
          aStringFromInt(num, pSession->nSessionPID);
          aStringCat(breakpoint, num);
          aStringCat(breakpoint, ")'>");
        } else {
          aStringCopy(breakpoint, "&nbsp;");
        }
      }
      aStream_Write(aStreamLibRef(reply), reply, 
      		    breakpoint, 
      		    aStringLen(breakpoint),
      		    &dsErr);
      break;

    /* parameter ^1 is class of souce line (hilighting) */
    case 1:
      if (pSession->bRunning) {
        /* grey all code if process is running */
        aStream_Write(aStreamLibRef(reply), reply, 
      		      "g", 2, &dsErr);
      } else {
        /* highlight line where stopped */
        if ((pSession->nIndex + 1) == (int)pSession->nCurrentLine)
          aStream_Write(aStreamLibRef(reply), reply, 
      		      "sh", 2, &dsErr);
        else 
          aStream_Write(aStreamLibRef(reply), reply, 
      		      "s", 1, &dsErr);
      }
      break;

    /* parameter 2 is the display text line */
    case 2:
      {
        aTextDisplayType type;
        dsErr = aTextDisplay_NextEnum(pSession->output, 
        			      pSession->lineBuffer, 
      				      aTEADGBMAXDISPLAYWIDTH * 3, 
      				      &type);
	if (dsErr == aErrNone)
          aStream_Flush(aStreamLibRef(reply), 
          		      pSession->lineBuffer,
          		      reply, 
      		              &dsErr);
      }
      pSession->nIndex++;
      break;

    default:
      break;

    } /* switch */

  } else {
    /* signals the template stuff that we are done with the 
     * BLOCK and also clean up our state holders */
    pSession->nIndex = 0;
    pSession->output = NULL;
    dsErr = aErrEOF;
  }

  return dsErr;

} /* sRenderCodeLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderCode
 */

aErr aTEADebugSession_RenderCode(aTEADebugSession* pSession,
			         aStreamRef reply)
{
  aErr dsErr = aErrNone;
  aTEADebugger* pDebugger;

  aAssert(pSession);

  pDebugger = pSession->pDebugger;

  /* here, we should get the appropriate output for the mode
   * (source or assembly) from the session's settings */

  if (!pSession->sourceDisplayRef) {
    /* do lazy loading so we only get the parts people want to
     * see in the debugger */
    if (dsErr == aErrNone)
      dsErr = sCreateHTMLDisplayFromFile(pDebugger->ioRef,
    				         pSession->pSourceName,
    				         pSession->eSourceArea,
    				         &pSession->sourceDisplayRef,
    				         &pSession->nSourceLines);
  }
  if (dsErr == aErrNone)
    pSession->output = pSession->sourceDisplayRef;

  aAssert(pDebugger);
  aAssert(pSession);

  /* set up start and end lines for code render */
  {
    int line = (int)pSession->nCurrentLine;
    int half = ((int)pSession->nWindowLines / 2);
    int start = line - half;
    int end = line + half;
    int rendered;
    if (start < 0) {
      /* centered window starts before valid limit */
      /* render starting from first line */
      /* render enough lines to fill window */
      start = 0;
      end = (int)pSession->nWindowLines - 1;
    } else if (end > (int)(pSession->nSourceLines - 1)) {
      /* centered window ends after valid limit */
      /* render so last line is at bottom of window */
      /* then handle case if code is smaller than window */
      start = (int)pSession->nSourceLines 
      		   - (int)pSession->nWindowLines;
      end = (int)pSession->nSourceLines;
      if (start < 0) {
        start = 0;
        end = (int)pSession->nWindowLines - 1;
      }
    }
    pSession->nCodeRenderStart = (unsigned int)start;
    pSession->nCodeRenderEnd = (unsigned int)end;
    rendered = end - start + 1;
  }

  pSession->nIndex = (int)pSession->nCodeRenderStart;
  dsErr = aTextDisplay_PrepareEnum(pSession->output, 
  				   pSession->nCodeRenderStart, 1);

  if (dsErr == aErrNone) {
    aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;
    aHTTP_Template(pSession->pDebugger->uiRef, 
  		   pConsole->http,
  		   "c.tpl",
  		   sRenderCodeLine,
  		   (void*)pSession,
  		   reply,
  		   &dsErr);
  }

  /* code is fresh */
  pSession->nDirtyFrames &= !aDBG_DIRTY_CODE;

  return dsErr;

} /* aTEADebugSession_RenderCode */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderStack
 */

aErr sRenderStack(const unsigned int nParamIndex,
		  const unsigned int nBlockIndex,
		  aStreamRef reply,
		  void* vpRef)
{
  aErr dsErr = aErrNone;
  char buf[10];
  aBool bDone = aFalse;

  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;

  if (pSession->nIndex >= -1) {
    switch (nParamIndex) {

    /* parameter ^0 is the hilighting */
    case 0:
      if (pSession->nIndex < 0) {
        bDone = aTrue;
      } else {
      
        if (pSession->bRunning) {
          /* grey all stack entries if process is running */
          aStringCopy(buf, "g");
        } else {
          /* highlight changes */
          if (pSession->fStackChg[pSession->nIndex])
            aStringCopy(buf, "sh");
          else 
            aStringCopy(buf, "s");
        }
      }
      break;

    /* parameter ^1 is the height */
    case 1:
      aStringFromInt(buf, pSession->sp - pSession->nIndex);
      break;

    /* parameter ^2 is the formated value */
    case 2:
      sFormatStackByte(buf, pSession, pSession->nIndex);
      break;

    /* parameter ^3 is the depth */
    case 3:
      aStringFromInt(buf, pSession->nIndex);
      pSession->nIndex--;
      break;

    default:
      *buf = 0;
      break;

    } /* switch */

    if (*buf != 0) {
      aStream_Write(aStreamLibRef(reply), reply, 
      		    buf, aStringLen(buf), &dsErr);
    }
  }
  
  if (bDone && (dsErr == aErrNone))
    dsErr = aErrEOF;

  return dsErr;

} /* sRenderStack */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderStack
 */

aErr aTEADebugSession_RenderStack(aTEADebugSession* pSession,
			         aStreamRef reply)
{
  aErr dsErr = aErrNone;
  
  aAssert(pSession);

  pSession->nIndex = pSession->sp - 1;

  if (pSession->nIndex >= 0) {
    aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;
    aHTTP_Template(pSession->pDebugger->uiRef, 
  		   pConsole->http,
  		   "k.tpl",
  		   sRenderStack,
  		   (void*)pSession,
  		   reply,
  		   &dsErr);
  }

  /* stack is fresh */
  pSession->nDirtyFrames &= !aDBG_DIRTY_STACK;
  
  return dsErr;

} /* aTEADebugSession_RenderStack */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderVariable
 */

aErr sRenderVariable(const unsigned int nParamIndex,
		     const unsigned int nBlockIndex,
		     aStreamRef reply,
		     void* vpRef)
{
  aErr dsErr = aErrNone;
  char num[10];

  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;

  /* faked for now */
  char* names[2] = {"(n/a)", "(n/a)"};
  int vals[2] = {0, 0};
  char* format = "H|D|A";

  if (pSession->nIndex < 2) {
    switch (nParamIndex) {

    /* parameter ^0 is the variable name */
    case 0:
      aStream_Write(aStreamLibRef(reply), reply, 
      		    names[pSession->nIndex], 
      		    aStringLen(names[pSession->nIndex]),
      		    &dsErr);
      break;

    /* parameter ^1 is the variable value */
    case 1:
      aStringFromInt(num, vals[pSession->nIndex]);
      aStream_Write(aStreamLibRef(reply), reply, 
      		    num, aStringLen(num),
      		    &dsErr);
      break;

    case 2:
      aStream_Write(aStreamLibRef(reply), reply, 
      		    format, aStringLen(format),
      		    &dsErr);
      pSession->nIndex++;
      break;

    default:
      break;

    } /* switch */

  } else {
    /* signals the template stuff that we are done with the block */
    dsErr = aErrEOF;
  }

  return dsErr;

} /* sRenderVariable */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderVariables
 */

aErr aTEADebugSession_RenderVariables(aTEADebugSession* pSession, 
			              aStreamRef browser)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;

  aAssert(pSession);

  pSession->nIndex = 0;
  
  aHTTP_Template(pSession->pDebugger->uiRef, 
  		 pConsole->http,
  		 "v.tpl",
  		 sRenderVariable,
  		 (void*)pSession,
  		 browser,
  		 &dsErr);

  /* variables are fresh */
  pSession->nDirtyFrames &= !aDBG_DIRTY_VARS;
  
  return dsErr;

} /* aTEADebugSession_RenderVariables */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderCallStack
 */

aErr sRenderCallStack(const unsigned int nParamIndex,
		     const unsigned int nBlockIndex,
		     aStreamRef reply,
		     void* vpRef)
{
  aErr dsErr = aErrNone;
  char line[10];
  char num[10];

  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;
  aTEADebugStackFrame* pFrame = (aTEADebugStackFrame*)pSession->vpRef;

  if (pFrame) {
    switch (nParamIndex) {

    /* parameter ^0 is the address */
    case 0:
      aStringCopy(line, "00000");
      aStringFromInt(num, pFrame->address);
      aStringCopy(&line[5 - aStringLen(num)], num);
      aStream_Write(aStreamLibRef(reply), reply, 
      		    line, aStringLen(line),
      		    &dsErr);
      break;

    /* parameter ^1 is the subroutine name */
    case 1:
      if (pFrame->pName) {
        aStream_Write(aStreamLibRef(reply), reply, 
      		      pFrame->pName, 
      		      aStringLen(pFrame->pName),
      		      &dsErr);
      } else {
        aStream_Write(aStreamLibRef(reply), reply, 
      		      "&nbsp;", 6,
      		      &dsErr);
      }
      pSession->vpRef = pFrame->pNext;
      break;

    default:
      break;

    } /* switch */

  } else {
    /* signals the template stuff that we are done with the block */
    dsErr = aErrEOF;
  }

  return dsErr;

} /* sRenderCallStack */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderCallStack
 */

aErr aTEADebugSession_RenderCallStack(aTEADebugSession* pSession, 
			              aStreamRef browser)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;

  aAssert(pSession);

  /* set up a pointer to walk the list in the callbacks */
  pSession->vpRef = pSession->pStackFrames;

  aHTTP_Template(pSession->pDebugger->uiRef, 
  		 pConsole->http,
  		 "s.tpl",
  		 sRenderCallStack,
  		 (void*)pSession,
  		 browser,
  		 &dsErr);

  /* call stack is fresh */
  pSession->nDirtyFrames &= !aDBG_DIRTY_CALLS;
  
  return dsErr;

} /* aTEADebugSession_RenderCallStack */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderRegisters
 */

aErr sRenderRegisters(const unsigned int nParamIndex,
		     const unsigned int nBlockIndex,
		     aStreamRef reply,
		     void* vpRef)
{
  aErr dsErr = aErrNone;
  char num[10];

  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;

  /* keep it simple for now */
  char* names[6] = {"PC", "SP", "C", "N", "Z", "Err"};

  if (pSession->nIndex < 6) {
    switch (nParamIndex) {

    /* parameter ^0 is the variable name */
    case 0:
      aStream_Write(aStreamLibRef(reply), reply, 
      		    names[pSession->nIndex], 
      		    aStringLen(names[pSession->nIndex]),
      		    &dsErr);
      break;

    /* parameter ^1 is the variable value */
    case 1:
      aStringFromInt(num, pSession->nRegInfo[pSession->nIndex]);
      aStream_Write(aStreamLibRef(reply), reply, 
      		    num, aStringLen(num),
      		    &dsErr);
      pSession->nIndex++;
      break;

    default:
      break;

    } /* switch */

  } else {
    /* signals the template stuff that we are done with the block */
    dsErr = aErrEOF;
  }

  return dsErr;

} /* sRenderRegisters */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderRegisters
 */

aErr aTEADebugSession_RenderRegisters(aTEADebugSession* pSession, 
			              aStreamRef browser)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;

  aAssert(pSession);

  pSession->nIndex = 0;
  
  aHTTP_Template(pSession->pDebugger->uiRef, 
  		 pConsole->http,
  		 "r.tpl",
  		 sRenderRegisters,
  		 (void*)pSession,
  		 browser,
  		 &dsErr);

  /* registers are fresh */
  pSession->nDirtyFrames &= !aDBG_DIRTY_REGS;
  
  return dsErr;

} /* aTEADebugSession_RenderRegisters */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderButtons
 */

aErr sRenderButtons(const unsigned int nParamIndex,
		    const unsigned int nBlockIndex,
		    aStreamRef reply,
		    void* vpRef)
{
  aErr dsErr = aErrNone;
  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;
  char bBuff[400];
  char num[20];

/*  char* cHtmlForKill = "top.document.location=\"/aAsset/x.html\";\n";*/
  char* cHtmlForKill = "top.document.location=\"/debugger\";\n";


  /* these button names must coincide with the enabled button bits */
  char* sButtonName[] = {  
    opGoName,
    opStepInName,
    opStepOutName,
    opStepOverName,
    opStopName,
    opKillName
  };

  /* these view names must coincide with the dirty bits */
  char* views[] = {"l", "c", "k", "s", "r", "b"};


  if (dsErr == aErrNone) { 

    switch (nBlockIndex) {
    
    /* this is the main block */
    case 0:

      /* add in the correct session id for all urls */
      if (nParamIndex == 0) {
        aStringFromInt(num, pSession->nSessionPID);
        aStream_Write(aStreamLibRef(reply), reply,
      		      num, aStringLen(num),
      		      &dsErr);
      }
      break;

    /* this block handles which views should be redrawn */
    case 1:
    

      if (pSession->bPrepareToKill) {

        if (pSession->nIndex >= 1) {
          /* show block done */
          pSession->nIndex = 0;
          dsErr = aErrEOF;
        } else {
          aStream_Write(aStreamLibRef(reply), reply,
        	        cHtmlForKill,
        	        aStringLen(cHtmlForKill),
        	        &dsErr);
          pSession->nIndex++;
        }
      
      } else {

        if (pSession->nIndex >= 5) {

          pSession->nIndex = 0;
          dsErr = aErrEOF;

        } else {

          /* there are 5 possible views to redraw, only draw 
           * the dirty ones */
          while ((pSession->nIndex < 5)
                 && !((1 << pSession->nIndex) & pSession->nDirtyFrames))
            pSession->nIndex++;
          
          /* if we make it here with nIndex < 5, we hav a dirty view */
          if (pSession->nIndex < 5) {
        
            /* we only have one parameter here */
            aStringCopy(bBuff, "top.frames.");
            aStringCat(bBuff, views[pSession->nIndex]);
            aStringCat(bBuff, ".document.location = \"/");
            aStringCat(bBuff, views[pSession->nIndex]);
            aStringCat(bBuff, "?id=");
            aStringFromInt(num, pSession->nSessionPID);
            aStringCat(bBuff, num);
            aStringCat(bBuff, "\";\n");
            aStream_Write(aStreamLibRef(reply), reply,
        		        bBuff, aStringLen(bBuff),
        		        &dsErr);
          }

          pSession->nIndex++;        		        

        }
      }

      break;

    /* this block handles the drawing of the buttons */
    case 2:

      if (pSession->nIndex == 6) {
        pSession->nIndex = 0;
        dsErr = aErrEOF;
      } else {

        switch (nParamIndex) {

          /* parameter ^0 is the button */
          case 0:
            if (sActiveButton(pSession)) {
	      aStringCopy(bBuff, "<FORM METHOD='POST' ACTION='/b'>"
		                "<INPUT TYPE='HIDDEN' NAME='op' VALUE='");
              aStringCat(bBuff, sButtonName[pSession->nIndex]);
  	      aStringCat(bBuff, "'><INPUT TYPE='HIDDEN' NAME='id' VALUE='");
              aStringFromInt(num, pSession->nSessionPID);
              aStringCat(bBuff, num);
  	      aStringCat(bBuff, "'><TD ALIGN='CENTER'><IMG SRC='/aAsset/n");
              aStringCat(bBuff, sButtonName[pSession->nIndex]);
  	      aStringCat(bBuff, ".jpg' "
  				"onmouseover='src=\"/aAsset/h");
              aStringCat(bBuff, sButtonName[pSession->nIndex]);
	      aStringCat(bBuff, ".jpg\"' onmouseout='src=\"/aAsset/n");
              aStringCat(bBuff, sButtonName[pSession->nIndex]);
  	      aStringCat(bBuff, ".jpg\"' onclick='submit()'");
              aStringCat(bBuff, " WIDTH='40' HEIGHT='40'>");
              aStringCat(bBuff, "</TD>");
  	      aStringCat(bBuff, "</FORM>");
            } else {
              aStringCopy(bBuff, "<TD ALIGN='CENTER'>");
              aStringCat(bBuff, "<IMG SRC='/aAsset/d");
              aStringCat(bBuff, sButtonName[pSession->nIndex]);
              aStringCat(bBuff, ".jpg' WIDTH='40' HEIGHT='40'>");
              aStringCat(bBuff, "</TD>");
            }
            aStream_Write(aStreamLibRef(reply), reply, 
      		          bBuff, aStringLen(bBuff),
      		          &dsErr);
            pSession->nIndex++;
            break;

          default:
            aAssert(0); /* we shouldn't get here */
            break;

        } /* switch */

      }
      break;
    
    default:
      aAssert(0); /* we shouldn't get here */
      
    } /* nBlockIndex switch */
  }

  return dsErr;

} /* sRenderButtons */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderButtons
 */

aErr aTEADebugSession_RenderButtons(aTEADebugSession* pSession, 
			            aStreamRef browser)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;

  aAssert(pSession);

  pSession->nIndex = 0;

  aHTTP_Template(pSession->pDebugger->uiRef, 
  		 pConsole->http,
  		 "b.tpl",
  		 sRenderButtons,
  		 (void*)pSession,
  		 browser,
  		 &dsErr);

  /* the buttons are fresh */
  pSession->nDirtyFrames &= !aDBG_DIRTY_BUTTONS;
  
  if (pSession->bPrepareToKill)
    pSession->bKillNow = aTrue;
  
  
  return dsErr;

} /* aTEADebugSession_RenderButtons */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderDebugger
 */

aErr sRenderDebugger(const unsigned int nParamIndex,
		     const unsigned int nBlockIndex,
		     aStreamRef reply,
		     void* vpRef)
{
  aErr dsErr = aErrNone;

  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;
  
  aAssert(pSession);

  /* add in the session id for all urls */
  if (nParamIndex == 0) {
    char num[10];
    aStringFromInt(num, pSession->nSessionPID);
    aStream_Write(aStreamLibRef(reply), reply,
      		  num, aStringLen(num),
      		  &dsErr);
  }

  return dsErr;

} /* sRenderDebugger */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderProcList
 */

aErr sRenderProcList(const unsigned int nParamIndex,
		     const unsigned int nBlockIndex,
		     aStreamRef reply,
		     void* vpRef)
{
  aErr dsErr = aErrNone;
  char line[100];
  char num[10];
  
  char* cLink1 = "ID =<A HREF=\"http://127.0.0.1:8080/debugger?id=";
  char* cLink2 = "\">";
  char* cLink3 = "</A>";

  aTEADebugSession* pSession;
  aTEADebugger* pDebugger = (aTEADebugger*)vpRef;
  
  aAssert(pDebugger);

  pSession = (aTEADebugSession*)pDebugger->vpRef;

  if (pSession) {

      aStringCopy(line, cLink1);
      aStringFromInt(num, pSession->nSessionPID);
      aStringCat(line, num);
      aStringCat(line, cLink2);
      aStringCat(line, num);
      aStringCat(line, cLink3);
      aStream_Write(aStreamLibRef(reply), reply, 
      		    line, aStringLen(line),
      		    &dsErr);

      pDebugger->vpRef = pSession->pNext;

  } else {

    /* signals the template stuff that we are done with the block */
    dsErr = aErrEOF;
  }

  return dsErr;

} /* sRenderProcList */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderDebugger
 */

aErr aTEADebugSession_RenderDebugger(aTEADebugger* pDebugger, 
				     aTEADebugSession* pSession, 
			             aStreamRef browser)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)pDebugger->vpConsole;
  
  if (pSession) {
    aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;

    /* live session view */  
    pSession->nIndex = 0;
    aHTTP_Template(pSession->pDebugger->uiRef, 
  		 pConsole->http,
  		 "debugger.tpl",
  		 sRenderDebugger,
  		 (void*)pSession,
  		 browser,
  		 &dsErr);

    /* the buttons are fresh */
    pSession->nDirtyFrames &= !aDBG_DIRTY_BUTTONS;
    
  } else {

    /* there is no session to display */  
    /* so display list of live debuggable processes */
    pDebugger->vpRef = (aTEADebugSession*)pDebugger->pSessions;
    aHTTP_Template(pDebugger->uiRef, 
  		 pConsole->http,
  		 "p.tpl",
  		 sRenderProcList,
  		 (void*)pDebugger,
  		 browser,
  		 &dsErr);
  }
  
  
  return dsErr;

} /* aTEADebugSession_RenderDebugger */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRenderLogo
 */

aErr sRenderLogo(const unsigned int nParamIndex,
		 const unsigned int nBlockIndex,
		 aStreamRef reply,
		 void* vpRef)
{
  aErr dsErr = aErrNone;
  char line[100];
  unsigned long now;

  aTEADebugSession* pSession = (aTEADebugSession*)vpRef;

  char* refresh = "<META HTTP-EQUIV='refresh' CONTENT=\"1; URL=/l?id=^0\">";

  char* dobuttons = "top.frames.b.document.location=\"/b?id=^0\";\n";

  if (pSession->nIndex == 0) {

    line[0] = 0;

    switch (nParamIndex) {

    /* continually refresh when running */
    case 0:
      if (pSession->bRunning) {
        aString_SetNumParam(line, refresh, 
        		    (int)pSession->nSessionPID);
	pSession->bRefreshing = aTrue;
      }
      break;

    /* when not running, force a button repaint */
    case 1:
      if (!pSession->bRunning && pSession->bRefreshing) {
        pSession->bRefreshing = aFalse;
        aString_SetNumParam(line, dobuttons, 
        		    (int)pSession->nSessionPID);
      }
      break;

    /* which logo picture to draw */
    case 2:
      if (!aIO_GetMSTicks(aStreamLibRef(reply), &now, NULL)) {
        now = (now / 1000) % 12;
        aStringFromInt(line, now);
      }
      pSession->nIndex++;
      break;
      
    } /* switch */
    
    /* if a substitution was made */
    if (line[0])
      aStream_Write(aStreamLibRef(reply), reply, line, 
        	    aStringLen(line), &dsErr);
  } else
    dsErr = aErrEOF;

  return dsErr;

} /* sRenderLogo */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEADebugSession_RenderLogo
 */

aErr aTEADebugSession_RenderLogo(aTEADebugSession* pSession, 
			         aStreamRef browser)
{
  aErr dsErr = aErrNone;
  aConsole* pConsole = (aConsole*)pSession->pDebugger->vpConsole;

  aAssert(pSession);
  
  pSession->nIndex = 0;

  aHTTP_Template(pSession->pDebugger->uiRef, 
  		 pConsole->http,
  		 "l.tpl",
  		 sRenderLogo,
  		 (void*)pSession,
  		 browser,
  		 &dsErr);
    
  return dsErr;

} /* aTEADebugSession_RenderLogo */



#endif /* aDEBUGGER */
