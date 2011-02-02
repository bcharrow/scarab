/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsole.h                                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent BrainStem	   */
/*		console object.					   */
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

#ifndef _aConsole_H_
#define _aConsole_H_

#include "aErr.h"
#include "aTextDisplay.h"
#include "aIO.h"
#include "aUI.h"
#include "aStem.h"
#include "aStemMsg.h"
#include "aTEAvm.h"
#include "aSteep.h"
#include "aLeaf.h"
#include "aStream_ConsoleOutput.h"
#include "aTEADebugger.h"
#include "aTEADebugSession.h"
#include "aCrypt.h"
#include "aStemKernel.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * UI dimensions
 */

#define aT_BORDER	    5
#define aT_BUTTONSPACE     20
#define aT_BUTTONWIDTH     30
#define aT_BUTTONHEIGHT    10
#define aT_WIDTH	  220
#ifdef aWINCE
#define aT_HEIGHT	  140
#else
#define aT_HEIGHT	  150
#endif
#define aT_INPUTHEIGHT     16
#define aT_STATUSHEIGHT    10
#define aT_HBDIAMETER	    8

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * output window display masks
 */

#define kDisplayStatus		1
#define kDisplayOutput		2
#define kDisplayInput		4
#define	kDisplayCommand		8
#define kDisplayMessage	       16

#define kDisplayAll		kDisplayStatus                       \
				| kDisplayOutput                     \
				| kDisplayInput                      \
				| kDisplayCommand		     \
				| kDisplayMessage

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * output window size specs
 */

#define MAXINPUTLINE  	100
#ifdef aWINCE
#define NUMVISLINES	 10
#else
#define NUMVISLINES	 11
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * setting file specs
 */

#define SETTNGFILE		"console.config"
#define STARTUPFILE		"console.startup"
#define MODESETTING		"mode"
#define DEFAULTMODE		"brainstem"
#define MAXSETTING		32
#define DEBUGLEVELKEY		"debug"
#define DEFAULTDBGLEVEL		1
#define TIMEOUTKEY		"timeout"
#define	DEFAULTTIMEOUT		1000
#define WIDTHKEY		"width"
#define DEFAULTWIDTH		240
#define HEIGHTKEY		"height"
#define DEFAULTHEIGHT		240
#define LEFTKEY			"left"
#define DEFAULTLEFT		100
#define TOPKEY			"top"
#define DEFAULTTOP		100
#define BUFFERKEY		"outputBufferSize"
#define DEFAULTBUFFER		100
#define OUTCOLORKEY		"outputColor"
#define DEFAULTOUTCOLOR		0xFF0000
#define INCOLORKEY		"inputColor"
#define DEFAULTINCOLOR		0x00FF00
#define STATCOLORKEY		"statusColor"
#define DEFAULTSTATCOLOR	0x000000
#define CMDCOLORKEY		"commandColor"
#define DEFAULTCMDCOLOR		0x0000FF
#define MSGCOLORKEY		"messageColor"
#define DEFAULTMSGCOLOR		0x880088


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Here the possible console modes are defined
 */

#define modeBrainStem		0
#define modeSP03		1
#ifdef aTERMINAL
#define modeTerminal		2
#endif /* aTERMINAL */
#ifdef aCMUCAM
#define modeCMUCam		3
#endif /* aCMUCAM */

/* mode specific data */
typedef struct aConsoleTerminal {
  int			nTerminalFlags;
} aConsoleTerminal;

typedef struct aConsoleCMUCam {
  int			nVersion;
  int			cursorX;
  int			cursorY;
  int			cursorLeft;
  int			cursorTop;
  int			cursorRight;
  int			cursorBottom;
  int			cursorConfidence;
  unsigned char		thresholds[6];
  char			input[100];
  char			opName[30];
  int			nInput;
  char			operation[40];
} aConsoleCMUCam;

typedef struct aConsole* aConsolePtr;

/* mode callbacks */
typedef aErr (*modeInitProc)(aConsolePtr pConsole);
typedef aErr (*modeCleanupProc)(aConsolePtr pConsole);
typedef aErr (*modeDrawProc)(aConsolePtr pConsole);
typedef aErr (*modeSliceProc)(aConsolePtr pConsole);
typedef aErr (*modeInputProc)(aConsolePtr pConsole,
			      const char* pInputLine);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Here we define the basic header elements that need to 
 * be in all os-versions of the console structure.  This 
 * header needs to be the first element in the structure 
 * before any os-specific fields.  This is sort of like 
 * sub-classing without the C++ overhead.
 */

typedef struct aConsoleSP03 {
  unsigned char		nVolume;
  unsigned char		nPitch;
  unsigned char		nSpeed;
} aConsoleSP03;

typedef struct aConsoleVMCode {
  char* 		pStore;
  tADDRESS 		nSize;
  aTEAProcessID		nPID;
  struct aConsoleVMCode* pNext;
  struct aConsole*	pConsole;
} aConsoleVMCode;

#define aConsoleStructHeader				   	   \
  aIOLib		ioLib;					   \
  aUILib		uiLib;					   \
  aStemLib		stemLib;				   \
  aTEAvmLib		vmLib;					   \
  aSteepLib		steepLib;				   \
  aLeafLib		leafLib;				   \
  aSettingFileRef	settings;				   \
  aTextDisplayRef	outputRef;				   \
  aTextDisplayType	outputMask;				   \
  aStreamRef		linkStreamRef;				   \
  aStreamRef		relayStreamRef;				   \
  aStreamRef		consoleOutputRef;			   \
  aStreamRef		consoleMessageRef;			   \
  aStreamRef		displayBuf;                                \
  aStreamRef		lineBuffer;				   \
  aBool			bDone;					   \
  aBool			bShutdown;				   \
  aBool			bLEDOn;					   \
  int			nDebugLevel;				   \
  aTEADebugger*		pDebugger;				   \
  void*			testData;				   \
  char			userFName[CRYPTNAMELEN];		   \
  char			userLName[CRYPTNAMELEN];		   \
  char			userID[CRYPTIDLEN];			   \
  char			vendorName[CRYPTVENDORLEN];		   \
  aBool			bLicensed;				   \
  aStemKernel*		pKernel;				   \
  unsigned long		nMSTimeout;				   \
  unsigned int		nHistory;				   \
  int			nWidth;					   \
  int			nHeight;				   \
  int			nLeft;					   \
  int			nTop;					   \
  unsigned long		nCommandColor;				   \
  unsigned long		nInputColor;				   \
  unsigned long		nOutputColor;				   \
  unsigned long		nStatusColor;				   \
  unsigned long		nMessageColor;				   \
  unsigned int		nOutputLines;				   \
  unsigned int  	nOutputWidth;                              \
  								   \
  aHTTPRef		http;					   \
  								   \
  unsigned char		nMode;					   \
  union {							   \
    aConsoleTerminal	term;  				           \
    aConsoleCMUCam      cmucam;					   \
  } mode;   						           \
  modeInitProc		modeInit;				   \
  modeCleanupProc	modeCleanup;				   \
  modeDrawProc	        modeDraw;				   \
  aHTTPRequestProc	modeHTTP;				   \
  modeSliceProc		modeSlice;				   \
  modeInputProc		modeInput;				   \
  								   \
  aGDRef		drawGD;					   \
  unsigned int		drawWidth;				   \
  unsigned int		drawHeight;				   \
   								   \
  aConsoleSP03		rSP03;					   \
  								   \
  aConsoleVMCode* 	pVMCode;				   \
								   \
  int			check


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This is the basic os-independent console structure
 */

typedef struct aConsole {
  aConsoleStructHeader;
} aConsole;

#define aCONSOLECHECK  0xFADE

#define aVALIDCONSOLE(p) if(((p) == NULL) || 			   \
		(((aConsole*)p)->check != aCONSOLECHECK)) {	   \
		  consoleErr = aErrParam; }

			
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * "pure virtual methods" that need to be created for each 
 * operating system.
 */

aErr aConsole_Create(aConsole** ppConsole);
aErr aConsole_Destroy(aConsole* pConsole);
aErr aConsole_Run(aConsole* pConsole);
aErr aConsole_DisplayLine(aConsole* pConsole, 
			  const char* line,
			  const aTextDisplayType type);
aErr aConsole_UpdateHB(aConsole* pConsole);
aErr aConsole_CreateDrawPane(aConsole* pConsole,
			     const unsigned int width,
			     const unsigned int height);
aErr aConsole_DestroyDrawPane(aConsole* pConsole);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * Sets a timer that calls aConsole_Continue when
   * the time has elapsed.  The time (nTicks) is 
   * in 1/100 second increments.
   */

aErr aConsole_SetTimer(aConsole* pConsole, 
		       unsigned int pid,
		       unsigned int nTicks);
  
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * Sleeps in a processor friendly manner for the 
   * specified number of 1/100 second ticks.
   */

aErr aConsole_Sleep(aConsole* pConsole, 
		    unsigned int nTicks);
  


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * These are the os-independent routines that can be called
 * from any operating system.
 */

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * aConsole_Initialize is typically called from the 
   * os-specific aConsole_Create routine.
   */

aErr aConsole_Initialize(aConsole* pConsole);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * aConsole_SetupLink is called after the Console is initialized
   * to attempt to create the link to the Stem.
   */

aErr aConsole_SetupLink(aConsole* pConsole);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine turns the bNotDone flag to aFalse and
   * cleans up the os-independent structure elements.
   */

aErr aConsole_Shutdown(aConsole* pConsole);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * This routine allows the cross-platform console to handle
   * input and processing tasks.
   */

aBool aConsole_TimeSlice(aConsole* pConsole, 
			 aErr* pErr);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine takes an input line and handles it. The
   * line must be null terminated and shouldn't have any 
   * linefeed or carriage return characters at the end.
   */

aErr aConsole_HandleLine(aConsole* pConsole,
			 const char *line,
			 const aBool bDisplay);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine handles packets sent from the stem
   * it uses callbacks to update the display, etc.
   */

aErr aConsole_HandlePacket(aConsole* pConsole,
			   aPacketRef inPacketRef);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine handles a command
   */

aErr aConsole_ParseCommand(aConsole* pConsole,
			   aToken* pCommandToken,
			   aTokenizerRef tokenizer);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine handles a packet
   */

aErr aConsole_ParsePacket(aConsole* pConsole,
			  aToken* pFirstToken,
			  aTokenizerRef tokenizer,
			  aBool bDisplay);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine gathers raw data from an input stream
   * the data can be either integers (hex, binary, decimal)
   * or strings
   */

aErr aConsole_ParseRaw(aConsole* pConsole,
		       aTokenizerRef tokenizer,
		       char* data,
		       unsigned char* pDataLen,
		       unsigned char maxLen);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine handles input from a stream as command input
   */

aErr aConsole_CommandStream(aConsole* pConsole,
			    aStreamRef stream,
			    const aBool bDisplay);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine compiles based on flags and filenames
   */

aErr aConsole_Compile(aConsole* pConsole,
		      const int compileFlags,
		      aStreamRef out,
		      aStreamRef err,
		      const char* inputName,
		      const aFileArea eInputArea,
		      const char* outputName,
		      const aFileArea eOutputArea);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine compiles based on flags and filenames
   */

aErr aConsole_Leaf(aConsole* pConsole,
		   const int compileFlags,
		   aStreamRef out,
		   aStreamRef err,
		   const char* inputName,
		   const aFileArea eInputArea,
		   const char* outputName,
		   const aFileArea eOutputArea);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine launches base on filename
   */

aErr aConsole_Launch(aConsole* pConsole,
		     const char* fileName,
		     const aFileArea eArea,
		     const char* data,
		     const unsigned char dataLen,
		     aTEAVMExitProc exitProc,
		     const void* exitRef,
		     int flags,
		     aTEAProcessID* pid);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine is called whenever a VM needs code
   */

aErr aConsole_VMFetch(const aTEAProcessID nPID,
		      const tADDRESS nOffset,
		      char* pData,
		      const tADDRESS nDataSize,
		      void* ref);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine is called whenever a VM process exits
   */

aErr aConsole_VMExit(const aVMExit eExitCode,
		     const char* returnData,
		     const unsigned char returnDataSize,
		     const aTEAProcessID pid,
		     const void* ref);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine tries to load the named file into the 
   * module and slot
   */

aErr aConsole_Load(aConsole* pConsole,
		   aStreamRef fileStream,
		   unsigned char module,
		   unsigned char slot);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * callback routine for all virtual machine I/O port 
   * manipulation
   */

aErr aConsole_VMPortCB(aTEAProcessID pid,
		       tADDRESS port,
		       aBool bRead,
		       char* data,
		       tBYTE dataSize,
		       aTEAVMIOPortIOCallback portIO,
		       void* ref);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * handle display output from the VM's
   */

aErr aConsole_VMDisplay(aConsole* pConsole, 
			unsigned char module,
			aTEAProcessID pid,
			char* displayData);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * this routine loads a batch file and inputs it
   */

aErr aConsole_Batch(aConsole* pConsole,
		    const char* batchfile,
		    const aFileArea eArea,
		    const aBool bDisplay);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * displays version information
   */

aErr aConsole_ShowVersion(aConsole* pConsole,
			  const char* entity,
			  const unsigned long version);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * handle an initial batch file
   */

aErr aConsole_HandleStartupFile(aConsole* pConsole);


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * handle building up a line of text for output
   */

aErr aConsole_BufferOutput(aConsole* pConsole, 
			   const char* text);
aErr aConsole_DisplayBuffer(aConsole* pConsole, 
			    const aTextDisplayType type);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * refresh drawing pane
   */

aErr aConsole_UpdateDrawing(aConsole* pConsole);


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * handle drawing pane double click
   */

aErr aConsole_DrawingDoubleClick(aConsole* pConsole);

#endif /* _aConsole_H_ */
