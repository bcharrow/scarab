/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaScript.cpp                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GarciaScript application     //
//              object.                                            //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// Copyright 1994-2008. Acroname Inc.                              //
//                                                                 //
// This software is the property of Acroname Inc.  Any             //
// distribution, sale, transmission, or re-use of this code is     //
// strictly forbidden except with permission from Acroname Inc.    //
//                                                                 //
// To the full extent allowed by law, Acroname Inc. also excludes  //
// for itself and its suppliers any liability, wheither based in   //
// contract or tort (including negligence), for direct,            //
// incidental, consequential, indirect, special, or punitive       //
// damages of any kind, or for loss of revenue or profits, loss of //
// business, loss of information or data, or other financial loss  //
// arising out of or in connection with this software, even if     //
// Acroname Inc. has been advised of the possibility of such       //
// damages.                                                        //
//                                                                 //
// Acroname Inc.                                                   //
// www.acroname.com                                                //
// 720-564-0373                                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////

#include <math.h>
#include "acpGarciaScript.h"
#include "aUtil.h"



/////////////////////////////////////////////////////////////////////

aErr acpGarciaScriptHB::call()
{
  bool bHB = m_pcScript->m_garcia.getNamedValue(
  		"heartbeat-status")->getBoolVal();

  if (bHB != m_pcScript->m_bHB) {
    m_pcScript->m_bHB = bHB;
    m_pcScript->updateHeartbeat();
  }

  return aErrNone;
}



/////////////////////////////////////////////////////////////////////

acpGarciaScript::acpGarciaScript() :
  m_bFinalized(false),
  m_bHB(false),
  m_nTimerSetting(100),
  m_batteryCheck(0),
  m_lastStatus(0),
  m_eMode(kStarting)
{
  // configure heartbeat callback object
  acpValue hbProcVal(new acpGarciaScriptHB(this));
  m_garcia.setNamedValue("heartbeat-callback", &hbProcVal);

  if (aIO_GetLibRef(&m_ioRef, NULL))
     die("unable to open aIO library");
  
  if (aUI_GetLibRef(&m_uiRef, NULL))
     die("unable to open aUI library");

} // acpGarciaScript constructor



/////////////////////////////////////////////////////////////////////

acpGarciaScript::~acpGarciaScript()
{
  if (m_logView) {
    aStream_Destroy(m_uiRef, m_logView, NULL);
    m_logView = NULL;
  }
  if (m_ioRef) {
    aIO_ReleaseLibRef(m_ioRef, NULL);
    m_ioRef = NULL;
  }
  if (m_uiRef) {
    aUI_ReleaseLibRef(m_uiRef, NULL);
    m_uiRef = NULL;
  }

} // acpGarciaScript destructor



/////////////////////////////////////////////////////////////////////
// behavior completion callback

aErr acpGarciaScript::taskComplete (
  acpGarcia* pGarcia,
  acpObject* pBehavior
)
{
  char line[100];
  char num[10];

  acpGarciaScript* pScript = 
    (acpGarciaScript*)pGarcia->getNamedValue("app-object")->getVoidPtrVal();
  
  if (!pScript)
    return aErrUnknown;

  
  pScript->m_lastStatus = pBehavior->getNamedValue("completion-status")->getIntVal();

  // show some status
  aStringCopy(line, "script completed with status ");
  aStringFromInt(num, pScript->m_lastStatus);
  pScript->addLogLine(line);

  return aErrNone;

} // acpGarciaScript taskComplete method



/////////////////////////////////////////////////////////////////////

void acpGarciaScript::welcome()
{
  // show a welcome message
  addLogLine("Welcome to Garcia Script");
  addLogLine("");

} // acpGarciaScript welcome method



/////////////////////////////////////////////////////////////////////
// This routine gets called once for each timer interval to
// allow us to update the robot, feedback, and display states.

void acpGarciaScript::TimeSlice()
{
  // handle any final setup now that we are up and running
  if (!m_bFinalized) {
    acpValue logStreamVal(m_logView);
    m_garcia.setNamedValue("status-stream", &logStreamVal);
    m_garcia.setNamedValue("error-stream", &logStreamVal);
    m_bFinalized = true;
  }

  // check for script completion
  m_garcia.handleCallbacks(5);

  // change out the UI to reflect the current state
  if (m_garcia.getNamedValue("active")->getBoolVal()) {

    // we are switching to active
    if (m_eMode == kInactive) {
      enableButton(aCMD_DOSCRIPT, true);
      enableButton(aCMD_DOHALT, false);
      setStatusText("Garcia is Idle");
      m_eMode = kIdle;

    // else we are already active
    } else {

      // find out if we are running currently
      eMode eCurrent = (m_garcia.getNamedValue("idle")->getBoolVal()) ? 
      			kIdle : kRunning;

      switch (eCurrent) {

      case kIdle:
        // did we just became idle ?
        if (m_eMode != kIdle) {
          enableButton(aCMD_DOSCRIPT, true);
          enableButton(aCMD_DOHALT, false);
          setStatusText("Garcia is Idle");
          m_eMode = kIdle;
        }
        break;

      case kRunning:
        // did we just start running ?
        if (m_eMode != kRunning) {
          enableButton(aCMD_DOSCRIPT, false);
          enableButton(aCMD_DOHALT, true);
          setStatusText("Garcia is Running");
          m_eMode = kRunning;
        }
        break;

      case kInactive:
      default:
        aAssert(0); // shouldn't get here
        break;
      } // switch
    }
  } else {
    if (m_eMode != kInactive) {
      enableButton(aCMD_DOSCRIPT, false);
      enableButton(aCMD_DOHALT, false);
      setStatusText("Garcia is Inactive");
      m_eMode = kInactive;
    }
  }

} // acpGarciaScript TimeSlice method



/////////////////////////////////////////////////////////////////////
// acpGarciaScript::dialogIdle static method 
//
// Called periodically when the dialog for picking the script file
// is up.

aErr acpGarciaScript::dialogIdle(
  const void* ref
)
{
  acpGarciaScript* pScript = (acpGarciaScript*)ref;

  aAssert(pScript);

  pScript->TimeSlice();
  
  return aErrNone;
  
} // acpGarciaScript::dialogIdle static method



/////////////////////////////////////////////////////////////////////
// win32_acpGarciaScript dumpFilter method
//
// used to display only the .xml files in the pick file dialog

aBool acpGarciaScript::dumpFilter(
  const char* pFilename,
  const unsigned long nSize
)  
{
  char extension[aFILE_NAMEMAXCHARS];
  
  aUtil_GetFileExtension(extension, pFilename, NULL);
  if (!aStringCompare(extension, ".xml"))
    return aTrue;
  
  return aFalse;

} // acpGarciaScript dumpFilter method



/////////////////////////////////////////////////////////////////////
// This routine launches the file dialog,
// gets a script name, and processes the file.

void acpGarciaScript::cmdScript()
{
  aErr err = aErrNone;
  char filename[aFILE_NAMEMAXCHARS];

  aDialog_PickFile(m_uiRef,
    		   "Please Select XML Script",
    		   filename,
    		   aFileAreaUser,
    		   dumpFilter,
    		   dialogIdle,
    		   this,
    		   &err);

  if ((err == aErrNone) && aStringCompare(filename, "")) {
    char line[100];
    acpObject* pTask;

    // show some status
    aStringCopy(line, "preparing script behavior for \"");
    aStringCat(line, filename);
    aStringCat(line, "\"");
    addLogLine(line);

    // fab up the script behavior
    pTask = m_garcia.createNamedBehavior("script", "gui_script");
    acpValue file(filename);
    pTask->setNamedValue("filename", &file);

    // show some status
    aStringCopy(line, "queueing script behavior \"");
    aStringCat(line, filename);
    aStringCat(line, "\"");
    addLogLine(line);

    m_garcia.queueBehavior(pTask);
  }

} // acpGarciaScript cmdScript method



/////////////////////////////////////////////////////////////////////
// This routine stops a currently running script.

void acpGarciaScript::cmdHalt()
{
  m_garcia.flushQueuedBehaviors();

} // acpGarciaScript cmdScript method



/////////////////////////////////////////////////////////////////////
// This routine launches a browser window pointing to the Garcia
// API's mini web server.

void acpGarciaScript::cmdAPIView()
{
  aErr err = aErrNone;
  unsigned long address;

  if (err == aErrNone)
    aIO_GetInetAddr(m_ioRef, &address, &err);

  if (err == aErrNone) {
    char url[100];
    char ip[20];
    aStringCopy(url, "http://");
    aUtil_FormatInetAddr(ip, address, NULL);
    aStringCat(url, ip);
    aStringCat(url, ":8000/apiview");    
    aBrowser_LaunchURL(m_uiRef, url, &err);
  }

  aAssert(err == aErrNone);

} // acpGarciaScript cmdAPIView method



/////////////////////////////////////////////////////////////////////
// This routine dispatches the command the proper handling routine.

void acpGarciaScript::dispatchCommand(const int nCmd)
{
  switch (nCmd) {
 
  case aCMD_DOSCRIPT:
    cmdScript();
    break;

  case aCMD_DOHALT:
    cmdHalt();
    break;

  case aCMD_DOAPIVIEW:
    cmdAPIView();
    break;

  case aCMD_DOEXIT:
    cmdExit();
    break;

  } // switch

} // acpGarciaScript dispatchCommand


