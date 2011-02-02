/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aConsole.c                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of the unix BrainStem console 	   */
/*		specifics.  This is a command line version.	   */
/*              It relies on the curses library.                   */
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

#include <curses.h>

#include "aMemLeakDebug.h"
#include "aConsoleText.h"
#include "unix_aConsole.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * debugging info for when we are headless (without a window server)
 */

#ifdef aDEBUG
FILE* debug = 0;
static void dbg(const char* msg);
void dbg(const char* msg)
{
  if (!debug) {
    debug = fopen("debug.txt", "w");
  }
  fprintf(debug, "%s\n", msg);
}
#else /* aDEBUG */
#define dbg(msg);
#endif /* aDEBUG */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes 
 */

static void sDrawStatusBar(aUnixConsole* pUnixConsole);
static void sMapColor(const short index,
		      const unsigned long color);
static void sDrawOutput(aUnixConsole* pUnixConsole);
static aErr sHandleChar(aUnixConsole* pUnixConsole, 
			char c);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sDrawOutput
 */

void sDrawStatusBar(aUnixConsole* pUnixConsole)
{
  int i, len;

  move(LINES - 2, 0);
  attron(A_REVERSE);
  if (pUnixConsole->bHasColor)
    color_set(kPairBar, NULL);
  len = aStringLen(aT_PROD_NAME_TXT) + 1;
  for (i = 0; i < COLS - len; i++)
    printw(" ");
  printw(aT_PROD_NAME_TXT);
  printw(" ");
  attroff(A_REVERSE);
  refresh();

} /* sDrawStatusBar */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sMapColor
 */

void sMapColor(const short index,
	       const unsigned long color)
{
  float r = (float)((color & 0xFF0000) >> 16) / 255.0f;
  float g = (float)((color & 0xFF00) >> 8) / 255.0f;
  float b = (float)(color & 0xFF) / 255.0f;

  init_color(index, (short)(r * 1000),
	     (short)(g * 1000),
	     (short)(b * 1000));

} /* sMapColor */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sDrawOutput
 */

void sDrawOutput(aUnixConsole* pUnixConsole)
{
  aErr consoleErr = aErrNone;
  unsigned int i;
  unsigned int top;
  int x, y;

  aVALIDCONSOLE(pUnixConsole);

  /* don't try to draw if we aren't initialized yet */
  if ((consoleErr != aErrNone) || !pUnixConsole->lineBuffer)
    return;

  /* get the current cursor position */
  getyx(stdscr, y, x);
 
  /* start by erasing the background */
  if (pUnixConsole->bHasColor)
    color_set(kPairBar, NULL);
  for (i = 0; i < pUnixConsole->nOutputLines; i++) {
    int t = i;
    move(t, 0);
    printw(pUnixConsole->pBlankOutputLine);
  }

  /* find the number of lines in the current display */
  if (consoleErr == aErrNone) {
    consoleErr = aTextDisplay_GetDisplayLines(pUnixConsole->outputRef,
					      &top, kDisplayAll);
    if (top > pUnixConsole->nOutputLines) {
      top = top - pUnixConsole->nOutputLines;
    } else {
      top = 0;
    }
  }
  
  /* draw the new text */
  if (consoleErr == aErrNone)
    consoleErr = aTextDisplay_PrepareEnum(pUnixConsole->outputRef,
  					  top, 
  					  pUnixConsole->outputMask);  

  for (i = 0; (consoleErr == aErrNone)
              && (i < pUnixConsole->nOutputLines); i++) {
    unsigned int t;
    aTextDisplayType type;
    aTextDisplay_NextEnum(pUnixConsole->outputRef, 
			  pUnixConsole->lineBuffer, 
			  pUnixConsole->nOutputWidth,
			  &type);

    if (pUnixConsole->bHasColor) {
      switch (type) {
      case kDisplayStatus:
        color_set(kPairStatus, NULL);
        break;
      case kDisplayInput:
        color_set(kPairInput, NULL);
        break;
      case kDisplayOutput:
        color_set(kPairOutput, NULL);
        break;
      case kDisplayCommand:
        color_set(kPairCommand, NULL);
        break;
      case kDisplayMessage:
        color_set(kPairMessage, NULL);
        break;
      } /* switch */
    }

    /* get the characters for the line */
    if (consoleErr == aErrNone) {
      char* pChars;
      aMemSize lineLen;
      aStreamBuffer_Get(pUnixConsole->ioLib,
			pUnixConsole->lineBuffer,
			&lineLen, &pChars, &consoleErr);

      /* draw the characters */
      if ((consoleErr == aErrNone) && lineLen) {
        t = i;
        move(t, 0);
        printw(pChars);
      }
    }

  } /* for */

  /* restore the cursor position */
  move(y, x);

  refresh();

} /* sDrawOutput */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sHandleChar
 */

aErr sHandleChar(aUnixConsole* pUnixConsole, 
		 char c)
{
  aErr consoleErr = aErrNone;
  int y, y1;

  switch (c) {

  case 13:
    /* null terminate */
    pUnixConsole->input[pUnixConsole->cur] = 0;
    dbg(pUnixConsole->input);
    consoleErr = aConsole_HandleLine((aConsole*)pUnixConsole,
      				     pUnixConsole->input,
      				     aTrue);

    /* reset the current input line to empty */
    pUnixConsole->input[0] = 0;
    pUnixConsole->cur = 0;
    y1 = y = LINES - 1;

    /* start with an empty line */
    if (pUnixConsole->bHasColor)
      color_set(kPairStatus, NULL);
 
    /* draw the empty line to erase previous input */
    move(y, 0);
    printw(pUnixConsole->pBlankOutputLine);

    /* move the cursor back to the start of the empty line */
    move(y1, 0);
    refresh();
    break;

  default:
    /* add the character to the buffer */
    pUnixConsole->input[pUnixConsole->cur++] = c;
    pUnixConsole->input[pUnixConsole->cur] = 0;
    break;

  } /* switch c */
  
  return consoleErr;

} /* sHandleChar */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Create
 */

aErr aConsole_Create(aConsole** ppConsole)
{
  aErr consoleErr = aErrNone;
  aUnixConsole* pUnixConsole = NULL;
  int i;

  if (ppConsole == NULL)
    consoleErr = aErrParam;

  /* initialize return value */
  if (consoleErr == aErrNone)
    *ppConsole = NULL;

  /* build storage for the main console application object */  
  if (consoleErr == aErrNone) {
    pUnixConsole = (aUnixConsole*)aMemAlloc(sizeof(aUnixConsole));
    if (pUnixConsole == NULL)
      consoleErr = aErrMemory;
    else
      aBZero(pUnixConsole, sizeof(aUnixConsole));
  }

  /* initialize curses before initializing the console object as
   * the console initialization can display text to the output */
  if (consoleErr == aErrNone) {
    dbg("setting up curses");
    initscr();
    start_color();
    cbreak();
    //    noecho();
    //    if (has_colors())
    //  pUnixConsole->bHasColor = aTrue;

    nonl(); 
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);

    nodelay(stdscr, TRUE);
  }

  /* initialize the dimensions based on the size of our display */  
  if (consoleErr == aErrNone) {
    pUnixConsole->nOutputLines = LINES - 2;
    pUnixConsole->nOutputWidth = COLS;
  }

  /* build an empty buffer for efficient erasing of the input and output */
  if (consoleErr == aErrNone) {
    pUnixConsole->pBlankOutputLine = 
    	(char*)aMemAlloc(sizeof(char) 
    		         * (pUnixConsole->nOutputWidth + 1));
    if (pUnixConsole->pBlankOutputLine) {
      char* p = pUnixConsole->pBlankOutputLine;
      for (i = 0; i < (int)pUnixConsole->nOutputWidth; i++)
        *p++ = ' ';
      *p = 0;
    } else {
      consoleErr = aErrMemory;
    }
  }

  /* call the initialize routine for the console application */
  if (consoleErr == aErrNone) {
    dbg("initializing console");
    consoleErr = aConsole_Initialize((aConsole*)pUnixConsole);
  }

  /* set up the link we will use for BrainStem communication */
  if (consoleErr == aErrNone) {
    dbg("setting up link");
    consoleErr = aConsole_SetupLink((aConsole*)pUnixConsole);
  }

  /* set up the colors when supported and initialized from settings */
  if ((consoleErr == aErrNone) && pUnixConsole->bHasColor) {
    if (can_change_color()) {
      sMapColor(kHBColor, 0xFF00);
      sMapColor(kBGColor, 0xFFFFFF);
      sMapColor(kAcroColor, 0xCCCC99);
      sMapColor(kCommandColor, pUnixConsole->nCommandColor);
      sMapColor(kInputColor, pUnixConsole->nInputColor);
      sMapColor(kOutputColor, pUnixConsole->nOutputColor);
      sMapColor(kStatusColor, pUnixConsole->nStatusColor);
      sMapColor(kMessageColor, pUnixConsole->nMessageColor);
      init_pair(kPairHB, kHBColor, kAcroColor);
      init_pair(kPairBar, kStatusColor, kAcroColor);
      init_pair(kPairCommand, kCommandColor, kBGColor);
      init_pair(kPairInput, kInputColor, kBGColor);
      init_pair(kPairOutput, kOutputColor, kBGColor);
      init_pair(kPairStatus, kStatusColor, kBGColor);
      init_pair(kPairMessage, kMessageColor, kBGColor);
    } else {
      init_pair(kPairHB, COLOR_GREEN, COLOR_BLACK);
      init_pair(kPairBar, COLOR_WHITE, COLOR_BLACK);
      init_pair(kPairCommand, COLOR_BLUE, COLOR_WHITE);
      init_pair(kPairInput, COLOR_GREEN, COLOR_WHITE);
      init_pair(kPairOutput, COLOR_RED, COLOR_WHITE);
      init_pair(kPairStatus, COLOR_BLACK, COLOR_WHITE);
      init_pair(kPairMessage, COLOR_CYAN, COLOR_WHITE);
    }
  }

  /* draw the intial status bar */
  if (consoleErr == aErrNone)
    sDrawStatusBar(pUnixConsole);

  /* if everything worked, set the return parameter pointer */
  if (consoleErr == aErrNone)
    *ppConsole = (aConsole*)pUnixConsole;
  else {
    /* otherwise, clean up */
    if (pUnixConsole)
      aConsole_Destroy((aConsole*)pUnixConsole);
  }

  return consoleErr;

} /* aConsole_Create */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Run
 */

aErr aConsole_Run(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  aUnixConsole* pUnixConsole = (aUnixConsole*)pConsole;
  int c;

  /* the message loop */
  while ((!pConsole->bDone) && (consoleErr == aErrNone)) {

    if (pUnixConsole->bHasColor)
      color_set(kPairStatus, NULL);
   
    c = getch();
    if (c != ERR) {
      consoleErr = sHandleChar(pUnixConsole, (char)c);
      if (consoleErr)
	dbg("error in handle char");
    }


    if (consoleErr == aErrNone) {
      aConsole_TimeSlice(pConsole, &consoleErr);
      if (consoleErr) {
	dbg("error in time slice");
	consoleErr = aErrNone;
      }
    }
  }

  return consoleErr;

} /* aConsole_Run */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_DisplayLine
 */

aErr aConsole_DisplayLine(aConsole* pConsole, 
			  const char* line,
			  const aTextDisplayType type)
{
  aErr consoleErr = aErrNone;

  aUnixConsole* pUnixConsole = (aUnixConsole*)pConsole;

  aVALIDCONSOLE(pUnixConsole);

  /* add the output line to the output buffer */
  if (consoleErr == aErrNone)
    consoleErr = aTextDisplay_AddLine(pUnixConsole->outputRef, 
				      line, type);

  if (consoleErr == aErrNone)
    sDrawOutput(pUnixConsole);

  return consoleErr;

} /* end of aConsole_DisplayLine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_UpdateHB
 */

aErr aConsole_UpdateHB(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  aUnixConsole* pUnixConsole = (aUnixConsole*)pConsole;

  aVALIDCONSOLE(pUnixConsole);

  if (consoleErr == aErrNone) {
    int x, y, t;

    /* find the current cursor position */
    getyx(stdscr, y, x);
    t = LINES - 2;

    /* draw the heartbeat in the status bar */
    move(t, 1);
    if (pUnixConsole->bHasColor) {
      if (pConsole->bLEDOn) {
        color_set(kPairHB, NULL);
      } else {
        color_set(kPairBar, NULL);
      }
      printw("*");
    } else {
      attron(A_REVERSE);
      if (pConsole->bLEDOn)
        printw("*");
      else
        printw(" ");
      attroff(A_REVERSE);
    }

    /* restore the cursor position */
    move(y, x);
    refresh();
  }

  return consoleErr;

} /* end of aConsole_UpdateHB */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Update
 */

aErr aConsole_Update(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;

  if (consoleErr == aErrNone)
    sDrawOutput((aUnixConsole*)pConsole);

  return consoleErr;

} /* aConsole_Update */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_Destroy
 */

aErr aConsole_Destroy(aConsole* pConsole)
{
  aErr consoleErr = aErrNone;
  aUnixConsole* pUnixConsole = (aUnixConsole*)pConsole;

  aVALIDCONSOLE(pUnixConsole);

  if (pUnixConsole->pBlankOutputLine)
    aMemFree(pUnixConsole->pBlankOutputLine);

  if (consoleErr == aErrNone)
    consoleErr = aConsole_Shutdown(pConsole);

  /* thow away the actual console */
  if (consoleErr == aErrNone)
    aMemFree((aMemPtr)pUnixConsole);

  /* close curses */
  erase();
  refresh();
  endwin();

  return(consoleErr);

} /* aConsole_Destroy routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * main
 */

int main (int argc, char* argv[])
{
  aErr consoleErr = aErrNone;
  aConsole* pConsole;

  if (consoleErr == aErrNone) {
    dbg("creating console");
    consoleErr = aConsole_Create(&pConsole);
  }

  if (consoleErr == aErrNone) {
    dbg("running console");
    consoleErr = aConsole_Run(pConsole);
  }

  if (consoleErr == aErrNone) {
    dbg("destroying console");
    consoleErr = aConsole_Destroy(pConsole);
  }
  
  /* this gets compiled out for release builds.  Debug builds will
   * report memory leaks */
  aLeakCheckCleanup();

#ifdef aDEBUG
  if (debug)
    fclose(debug);
#endif /* aDEBUG */

  return(consoleErr);

} /* main */

#endif /* aUNIX */
