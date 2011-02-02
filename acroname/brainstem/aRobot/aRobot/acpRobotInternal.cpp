/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotInternal.cpp                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API library object.    //
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

#include "aCmd.tea"

#include "aUtil.h"
#include "aStreamUtil.h"

#include "acpException.h"
#include "acpPackageTagID.h"

#include "aRobotDefs.tea"
#include "acpRobotInternal.h"
#include "acpRobotPacketMessage.h"
//#include "acpRobotHTMLMain.h"
#include "aRobotProperties.h"
#include "acpRobotProperties.h"
#include "acpRobotUserObjects.h"

#include "acpRobotScript.h"
#include "acpRobotGlobal.h"
#include "acpRobotSleep.h"

#include "acpTag_apiPACKAGE.h"
#include "acpTag_PROPERTY.h"
#include "acpTag_PRIMITIVE.h"
#include "acpTag_TEMPLATE.h"
#include "acpTag_USEROBJECT.h"
#include "acpTag_SET.h"
#include "acpTag_GET.h"
#include "acpTag_INITIALIZER.h"
#include "acpTag_LINK.h"

#if defined(aWIN) || defined(aWINCE)
#include "win32_acpRobotOSFactory.h"
#endif // aWIN
#ifdef aUNIX
#include "unix_acpRobotOSFactory.h"
#endif // aUNIX



static float unitConvFac[aROBOT_UNITS_CODES][aROBOT_UNITS_CODES];

int lookupString(acpList<acpString>& rlist, acpString& rstr);

int lookupString(acpList<acpString>& rlist, acpString& rstr)
{
  acpListIterator<acpString> iter(rlist);
  acpString* pName;
  int k = 0;
  int nIndex = aROBOT_APITEA_NOTFOUND;
  while ((pName = iter.next())) {
    if (!aStringCompare((char*)(*pName), (char*)rstr)) {
      // zero-based
      nIndex = k;
    }
    k++;
  }
  return nIndex;
}



/////////////////////////////////////////////////////////////////////
// acpRobotInternal constructor
//

acpRobotInternal::acpRobotInternal (
  acpRobot* pcRobot,
  const char* pName
) :
  acpObject("robot", pName),
  m_ioRef(NULL),
  m_vmRef(NULL),
  m_uiRef(NULL),
  m_settings(NULL),
  m_pStems(NULL),
  m_pcViewSite(NULL),
  m_pcFactory(NULL),
  m_pcThread(NULL),
  m_bHB(false),
  m_statusStream(NULL),
  m_errorStream(NULL),
  m_pcRobot(pcRobot),
  m_nDistanceUnitType(aROBOT_UNITS_METERS),
  m_nAngleUnitType(aROBOT_UNITS_RADIANS),
  m_nMassUnitType(aROBOT_UNITS_KILOGRAMS),
  m_pCurrent(NULL),
  m_bActive(false),
  m_bSleeping(false),
  m_pnLastHB(NULL),
  m_nWakeUpTime(0L),
  m_nNextBehaviorID(100),
  m_pcHBCallback(NULL)
{
  aErr err;
  char* pcSetting;

  // initialize unit conversion matrix
  initConvFac();

  // set the main (robot) name to that of the API description
  // file minus the "_api".  For instance "Foo_api" is Foo.
  acpValue name("aRobot");
  if (*pName) {
    acpString robotName(pName);
    robotName.truncate("_api");
    name.set((char*)robotName);
  }
  setNamedValue("name", &name);

  // find the robot name from the passed-in api name

  // get the aIO library reference
  aIO_GetLibRef(&m_ioRef, &err);
  aAssert(err == aErrNone);

  // get the aUI library reference
  aUI_GetLibRef(&m_uiRef, &err);
  aAssert(err == aErrNone);

  // set up the API TEA machinery
  aTEAvm_GetLibRef(&m_vmRef, &err);
  aAssert(err == aErrNone);
  aTEAvm_Initialize(m_vmRef,
		    aROBOT_APITEA_STACKSIZE,
		    aROBOT_APITEA_PROCESSCT,
		    acpRobotVMManager::m_vmPortProc,
		    (void*)this,
		    &err);
  aAssert(err == aErrNone);
  for (int i = 0; i < aROBOT_APITEA_PROCESSCT; i++) {
    m_pcVMM[i] = new acpRobotVMManager(this, i);
  }

  // determine name of API being used
  // acpRobot self name determines name of config file
  // config file of a no-name API will be "robot.config"
  // otherwise it will be the API name followed by ".config"
  char sSettingFileName[aFILE_NAMEMAXCHARS];
  if (aStringLen(pName)) {
    aStringCopy(sSettingFileName, pName);
    aStringCat(sSettingFileName, ".config");
  } else {
    aStringCopy(sSettingFileName, aROBOT_SETTINGSFILE);
  }

  // get "global" settings
  if (!m_settings) {
    aSettingFile_Create(m_ioRef, 
			aROBOT_MAXSETTINGLEN,
			sSettingFileName,
			&m_settings, &err);
    aAssert(err == aErrNone);
  }

  // try to read the package
  acpRobotPackage* pPackage = NULL;
  listPkgTags tagList;
  if (pName) {
    char sPackageName[aFILE_NAMEMAXCHARS];
    aStringCopy(sPackageName, pName);
    if (aStringLen(sPackageName)) {
      aStringCat(sPackageName, ".robot");
      pPackage = readPackage(sPackageName);
      if (!pPackage) {
        // ???
//	m_pStem->DebugLine("Could not find API definition file.", NULL);
        aDialog_Message(m_uiRef,
	  "Could not find API definition file.",
	  NULL, NULL, NULL);
      }
    }
  }

  // if we have a package then slurp all the data out of it
  // this will also provide a list of the links used
  if (pPackage)
    collectTagsAndLinks(pPackage);

  // if no package, the used links list will be empty
  // in that case we assume default stream and add it
  if (m_listLinksUsed.length() == 0) {
    m_listLinksUsed.add(new acpString(""));
  }

  // we must build stem links before launching the thread
  // must allocate stem array and last heartbeat time array first
  m_numLinksUsed = m_listLinksUsed.length();
  m_pStems = (acpStem**)aMemAlloc(m_numLinksUsed * sizeof(acpStem*));
  int nHBArraySize = m_numLinksUsed * sizeof(unsigned long);
  m_pnLastHB = (unsigned long*)aMemAlloc(nHBArraySize);
  aBZero(m_pnLastHB, nHBArraySize );

  if (m_numLinksUsed) {
    for (int i = 0; i < m_numLinksUsed; i++) {

      char sSettingsFile[aFILE_NAMEMAXCHARS];
      acpString* pLinkName = m_listLinksUsed[i];
 
      if (pLinkName->length()) {
        aStringCopy(sSettingsFile, (char*)(*pLinkName));
        aStringCat(sSettingsFile, ".config");
      } else {
        if (aStringLen(pName)) {
          aStringCopy(sSettingsFile, pName);
          aStringCat(sSettingsFile, ".config");
        } else {
          aStringCopy(sSettingsFile, aROBOT_SETTINGSFILE);
        }
      }
      
      m_pStems[i] =
        new acpStem(
          m_ioRef,
          m_uiRef,
          sSettingsFile,
          sMultiHBProc,
          i,
          this,
          (char*)(*pLinkName));
    }
  }

  // build the debugging http server with its pages
/*
  m_pcViewSite = new acpHTMLSite(m_ioRef, m_uiRef, m_settings);
  m_pcViewSite->addPage("/apiview", 
  			new acpRobotHTMLMain(this));
  m_pcViewSite->addPage("/apiprimitives", 
  			new acpRobotHTMLPrimitives(this));
  m_pcViewSite->addPage("/apiprimitiveparams", 
  			new acpRobotHTMLPrimitiveParams(this));
  m_pcViewSite->addPage("/apiaddbehavior", 
  			new acpRobotHTMLAddBehavior(this));
*/
  // build the OS factory, this should be the only OS specific
  // code in the API.
#if defined(aWIN) || defined(aWINCE)
  m_pcFactory = new win32_acpRobotOSFactory();
#endif /* aWIN */
#ifdef aUNIX
  m_pcFactory = new unix_acpRobotOSFactory();
#endif /* aUNIX */    
  aAssert(m_pcFactory);

  
  // set up the global ack interval
  aAssert(m_settings);
  aSettingFile_GetULong(m_ioRef, m_settings,
  		            aROBOT_HBINTKEY,
  		            &m_nHBInterval, aROBOT_HBINTDEFAULT,
  		            &err);
  aAssert(err == aErrNone);


  // select global distance units
  aSettingFile_GetString(m_ioRef, m_settings,
  		            aROBOT_DISTUNITKEY,
  		            &pcSetting, aROBOT_TEXT_METERS,
  		            &err);
  aAssert(err == aErrNone);
  if (err == aErrNone) {
    acpValue distString(pcSetting);
    setNamedValue(aROBOT_PROPNAME_DISTUNITSSTR, &distString);
  }
    
  // select global angle units
  aSettingFile_GetString(m_ioRef, m_settings,
  		            aROBOT_ANGLEUNITKEY,
  		            &pcSetting, aROBOT_TEXT_RADIANS,
  		            &err);
  aAssert(err == aErrNone);
  if (err == aErrNone) {
    acpValue angleString(pcSetting);
    setNamedValue(aROBOT_PROPNAME_DISTUNITSSTR, &angleString);
  }


  m_pcBehaviorMutex = 
  	m_pcFactory->buildMutex("Behavior List Mutex");

  m_pcCallerMessageMutex = 
  	m_pcFactory->buildMutex("Caller Message Mutex");

  m_pcThread = m_pcFactory->buildThread("Robot API");
  m_pcThread->start(this);


  // global properties
  addProperty(new acpRobotActiveProperty(this));
  addProperty(new acpRobotIdleProperty(this));
  addProperty(new acpRobotStatusStreamProperty(this));
  addProperty(new acpRobotErrorStreamProperty(this));
  addProperty(new acpRobotDistanceUnitsProperty(this));
  addProperty(new acpRobotAngleUnitsProperty(this));
  addProperty(new acpRobotMassUnitsProperty(this));

  // heartbeat properties
  addProperty(new acpRobotHeartbeatCallbackProperty(this));
  addProperty(new acpRobotHeartbeatStatusProperty(this));

  // intrinsic primitives
  addSubObject(new acpRobotScript(this));
  addSubObject(new acpRobotGlobal(this));
  addSubObject(new acpRobotSleep(this));

  // build properties, primitives, and objects from package list
  // after everything else is done because initializers
  // need thread to be alive and running
  if (pPackage) {
    initTagData();
    delete pPackage;
  }

} // acpRobotInternal constructor



/////////////////////////////////////////////////////////////////////
// acpRobotInternal destructor
//

acpRobotInternal::~acpRobotInternal ()
{
  if (m_pcHBCallback)
    delete m_pcHBCallback;  
  if (m_pcThread)
    delete (m_pcThread);
  if (m_pcFactory)
    delete m_pcFactory;
  if (m_pcViewSite)
    delete m_pcViewSite;
  if (m_pcCallerMessageMutex)
    delete m_pcCallerMessageMutex;
  if (m_pcBehaviorMutex)
    delete m_pcBehaviorMutex;

  for (int i = 0; i < m_numLinksUsed; i++) {
    delete m_pStems[i];
  }
  aMemFree(m_pStems);
  if (m_pnLastHB)
    aMemFree(m_pnLastHB);

  if (m_settings)
    aSettingFile_Destroy(m_ioRef, m_settings, NULL);
  if (m_uiRef)
    aUI_ReleaseLibRef(m_uiRef, NULL);
  if (m_ioRef)
    aIO_ReleaseLibRef(m_ioRef, NULL);
  if (m_vmRef)
    aTEAvm_ReleaseLibRef(m_vmRef, NULL);

  for (int i = 0; i < aROBOT_APITEA_PROCESSCT; i++) {
    if (m_pcVMM[i])
      delete m_pcVMM[i];
  }

} // acpRobotInternal destructor



/////////////////////////////////////////////////////////////////////
// acpRobotInternal run method
//
// This is only to be called by the Robot private thread.
//

int acpRobotInternal::run()
{
  aAssert(m_pcThread->isThreadContext());
  
  while (!m_pcThread->isDone()) {
    bool bYield = true;

    // need to get the time
    aErr err;
    unsigned long now;
    bool bWakeUpNow = false;
    aIO_GetMSTicks(m_ioRef, &now, &err);
    aAssert(err == aErrNone);

    // if we're sleeping check for wakey-wakey time
    if (m_bSleeping && (now > m_nWakeUpTime)) {
      bWakeUpNow = true;
    }

    // check for packet timeouts in API TEA processes
    for (int i = 0; i < aROBOT_APITEA_PROCESSCT; i++)
      m_pcVMM[i]->checkTimeout(now);

    // see if link heartbeat has been lost in any Stem
    // else process any pending behaviors in the queue
    bool bLost = false;
    for (int i = 0; i < m_numLinksUsed; i++) {
      if (now > m_pnLastHB[i] + m_nHBInterval) {
        bLost = true;
        break;
      }
    }
    if (bLost) {
      m_bActive = false;
    } else {
      m_bActive = true;
      bool bRan = false;
      if (!m_pCurrent) {
        m_pcBehaviorMutex->lock();
        m_pCurrent = m_behaviors.removeHead();
        m_pcBehaviorMutex->unlock();
        if (m_pCurrent) {
          m_pCurrent->execute();
        }
      }
      // if nothing executed check if we are
      // sleeping and it is time to wake up
      if (!bRan) {
        if (m_bSleeping && bWakeUpNow) {
          m_bSleeping = false;
          finishBehavior(aROBOT_ERRFLAG_NORMAL);
        }
      }
    }

    // execute VM opcodes
    // if processes are busy then don't yield
#if 0
    // this could work if there are no I/O calls 
    // that could be pending
    aBool bBusy = aFalse;
    aErr e;
    aTEAvm_TimeSlice(m_vmRef, &bBusy, &e);
    aAssert(e == aErrNone);
    if (bBusy)
      bYield = false;
#else
    aTEAvm_TimeSlice(m_vmRef, NULL, NULL);
    bool bVMBusy = false;
    for (int i = 0; i < aROBOT_APITEA_PROCESSCT; i++)
      bVMBusy = bVMBusy || m_pcVMM[i]->isBusy();
    if (bVMBusy)
      bYield = false;
#endif

    // check for a message and handle it if there is one
    acpMessage* pMessage;
    pMessage = m_pcThread->getMessage();
    if (pMessage) {
      pMessage->process();
      delete pMessage;
      bYield = false;
    }

    // handle any http requests for the API View
    aBool bChanged = aFalse;
//???    m_pcViewSite->slice(bChanged);
    if (bChanged)
      bYield = false;

    // handle link activity
    for (int i = 0; i < m_numLinksUsed; i++) {

      acpStem* pStem = m_pStems[i];

      // drain packets until none are left
      // (a single call might miss packets)
      aPacketRef packet;
      aErr packetErr = aErrNone;
      while (packetErr == aErrNone) {    

        pStem->GetPacket(NULL,
			 NULL,
			 0,
			 &packet,
			 &packetErr);

        if (packetErr == aErrNone) {
          unsigned char address;
          unsigned char length;
          char data[aSTEMMAXPACKETBYTES];
          if (!pStem->GetData(packet, &address, &length, data, NULL))
            dispatchPacketData(address, length, data);
          pStem->DestroyPacket(packet, NULL);
          // if we got a packet, don't yield the thread
          bYield = false;
        }
      }
    }

    // yield for a bit so we don't swamp the processor when nothing
    // is going on
    if (bYield)
      m_pcThread->yield(aROBOT_YIELDMS);

  } // while 

  // The thread is going down right now!!!
  // Destroy all behaviors up front here so that we don't get
  // calls later in the destructors that rely on this thread
  while (m_behaviors.head()) {
    acpRobotBehavior* pBehavior;
    pBehavior = m_behaviors.removeTail();
    delete pBehavior;
  }

  // kill any current behavior that is running on the robot
  // to avoid any callbacks
  if (m_pCurrent) {
    m_pcVMM[aROBOT_APITEA_PRIMPROCID]->withinThreadKill();
    // stall to ensure the serial bytes get out
    aIO_MSSleep(m_ioRef, 100, NULL);
  }

  return 0;

} // acpRobotInternal run method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal addStatusLine method
//

void acpRobotInternal::addStatusLine(const char* pLine)
{
  if (m_statusStream) {
    addCallerMessage(
    	new acpStreamWriteLineMessage(m_statusStream, pLine));
  }
} // acpRobotInternal addStatusLine method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal statusString method
//

char* acpRobotInternal::statusString (
  const short nStatus,
  char* pText,
  unsigned int nMaxChars
)
{
  aAssert(pText);
  *pText = '\0';

  switch (nStatus) {

  case aROBOT_ERRFLAG_NORMAL:
    aStringNCopy(pText, "No Error", nMaxChars);
    break;

  case aROBOT_ERRFLAG_ABORT:
    aStringNCopy(pText, "Aborted", nMaxChars);
    break;

  case aROBOT_ERRFLAG_NOTEXECUTED:
    aStringNCopy(pText, "Not Executed", nMaxChars);
    break;

  case aROBOT_ERRFLAG_WONTEXECUTE:
    aStringNCopy(pText, "Will Not Execute", nMaxChars);
    break;

  case aROBOT_ERRFLAG_APIVMERROR:
    aStringNCopy(pText, "VM Error", nMaxChars);
    break;

  default:
    // look up in user maps (TBD) ???
    aStringNCopy(pText, "(status)", nMaxChars);
    break;

  } // nStatus switch

  return pText;

} // acpRobotInternal statusString method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal queueBehavior method
//
// This code can be called from any thread.

void acpRobotInternal::queueBehavior(acpRobotBehavior* pBehavior)
{ 
  aAssert(pBehavior);

  m_pcBehaviorMutex->lock();
  m_behaviors.addToTail(pBehavior);
  m_pcBehaviorMutex->unlock();

} // acpRobotInternal queueBehavior



/////////////////////////////////////////////////////////////////////
// acpRobotInternal queueBehaviorHead method
//
// This code can be called from any thread.

void acpRobotInternal::queueBehaviorHead(acpRobotBehavior* pBehavior)
{ 
  aAssert(pBehavior);

  m_pcBehaviorMutex->lock();
  m_behaviors.addToHead(pBehavior);
  m_pcBehaviorMutex->unlock();

} // acpRobotInternal queueBehaviorHead



/////////////////////////////////////////////////////////////////////
// acpRobotInternal addCallerMessage method
//

void acpRobotInternal::addCallerMessage (
  acpMessage* pMessage
)
{
  aAssert(pMessage);
  
  m_pcCallerMessageMutex->lock();
  m_callerMessages.addToTail(pMessage);
  m_pcCallerMessageMutex->unlock();

} // acpRobotInternal::addCallerMessage method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal isIdle method
//

bool acpRobotInternal::isIdle()
{
  return ((m_behaviors.length() == 0) && !m_pCurrent);

} // acpRobotInternal isIdle



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal goToSleep method
//

void acpRobotInternal::goToSleep (
  const unsigned long nMSSleep
)
{
  if (!m_bSleeping) {
    unsigned long now;
    aIO_GetMSTicks(m_ioRef, &now, NULL);
    m_nWakeUpTime = now + nMSSleep;
    m_bSleeping = true;
  }
} // acpRobotInternal goToSleep method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal createBehavior method
//

acpRobotBehavior* acpRobotInternal::createNamedBehavior (
  const char* pPrimitiveName,
  const char* pBehaviorName,
  acpRobotBehaviorList* pParent // = NULL
)
{
  acpRobotPrimitive* pPrimitive =
    (acpRobotPrimitive*)getNamedSubObject("primitive", pPrimitiveName);
  aAssert(pPrimitive);

  return pPrimitive->factoryBehavior(pBehaviorName, 
  				     pPrimitive, 
  			 	     m_nNextBehaviorID++, 
  			 	     this,
  			 	     pParent);

} // acpRobotInternal createBehavior method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal createBehavior method
//

acpRobotBehavior* acpRobotInternal::createBehavior (
  const int nPrimitiveIndex,
  const char* pBehaviorName,
  acpRobotBehaviorList* pParent // = NULL
)
{
  acpRobotPrimitive* pPrimitive =
    (acpRobotPrimitive*)getSubObject(nPrimitiveIndex);
  aAssert(pPrimitive);

  return pPrimitive->factoryBehavior(pBehaviorName, 
  				     pPrimitive, 
  			 	     m_nNextBehaviorID++, 
  			 	     this,
  			 	     pParent);

} // acpRobotInternal createBehavior method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal handlePacketOut method
//
// Must be called from the thread context.
//

void acpRobotInternal::handlePacketOut(acpRobotPacketMessage* pMessage)
{
  aAssert(m_pcThread->isThreadContext());

  aErr err;
  aPacketRef packet;
  m_pStems[pMessage->m_nLinkID]->
    CreatePacket(
      pMessage->m_address,
      pMessage->m_length,
      pMessage->m_data,
      &packet, &err);
  aAssert(err == aErrNone);
  
  m_pStems[pMessage->m_nLinkID]->SendPacket(packet, &err);
  aAssert(err == aErrNone);

} // acpRobotInternal handlePacketOut method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal sMultiHBProc method
//

aErr acpRobotInternal::sMultiHBProc (
  const aBool bHBOn,
  void* ref)
{
  acpStem* pStem = (acpStem*)ref;
  aAssert(pStem);

  acpRobotInternal* pRobotInternal =
    (acpRobotInternal*)pStem->getAppRef();

  aAssert(pRobotInternal);

  // record the time of the heartbeat
  aErr err;
  aIO_GetMSTicks(pRobotInternal->m_ioRef,
    		 &pRobotInternal->m_pnLastHB[pStem->getID()],
    		 &err);
  aAssert(err == aErrNone);

  // store the heartbeat state
  pRobotInternal->m_bHB = (bHBOn == aTrue);

  // now, message the callback if it is present
  if (pRobotInternal->m_pcHBCallback) {
    pRobotInternal->addCallerMessage(
    	new acpRobotMessage(pRobotInternal->m_pcRobot,
                             pRobotInternal->m_pcHBCallback));
  }

  return err;

} // acpRobotInternal sMultiHBProc routine



/////////////////////////////////////////////////////////////////////
// acpRobotInternal finishBehavior method
//
// can be called from anywhere
//

void acpRobotInternal::finishBehavior(const short status)
{
  aAssert(m_pcThread->isThreadContext());
  aAssert(m_pCurrent);

  m_pCurrent->finish(status, NULL);

} // acpRobotInternal finishBehavior



/////////////////////////////////////////////////////////////////////
// acpRobotInternal dispatchPacketData method
//
// Must be called from the thread context.
//

void acpRobotInternal::dispatchPacketData (
  unsigned char address,
  unsigned char length,
  char* data
)
{
  aAssert(m_pcThread->isThreadContext());

  if (!length)
    return;

  // see if an API TEA process is waiting for data
  for (int i = 0; i < aROBOT_APITEA_PROCESSCT; i++)
    m_pcVMM[i]->filterPacket(address, length, data);

} // dispatchPacketData



/////////////////////////////////////////////////////////////////////
// acpRobotInternal handleCallerMessages method
//
// Can be called from any thread context.
//

int acpRobotInternal::handleCallerMessages (
  const unsigned long nMSYield
)
{
  int nMessages = 0;
  unsigned long now = 0;
  unsigned long done = 0;


  if (nMSYield) {
    aIO_GetMSTicks(m_ioRef, &now, NULL);
    done = now + nMSYield;
  }

  do {
    // remove the behavior from the list
    while (!m_callerMessages.isEmpty()) {
      m_pcCallerMessageMutex->lock();
      acpMessage* pMessage = m_callerMessages.removeHead();
      m_pcCallerMessageMutex->unlock();
      nMessages++;
      pMessage->process();
      delete pMessage;
    }
    
    // sleep to avoid swamping the CPU
    aIO_MSSleep(m_ioRef, 1, NULL);

    if (done)
      aIO_GetMSTicks(m_ioRef, &now, NULL);

  } while (done > now);
  
  return nMessages;

} // acpRobotInternal handleCallerMessages method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal clearCurrent method

void acpRobotInternal::clearCurrent (
  acpRobotBehavior* pDead // = NULL
)
{
  if ((pDead == m_pCurrent) || !pDead) {
    m_pCurrent = NULL;
  }

} // acpRobotInternal clearCurrent



/////////////////////////////////////////////////////////////////////
// acpRobotInternal flushQueuedBehaviors method
//

void acpRobotInternal::flushQueuedBehaviors()
{
  aAssert(!m_pcThread->isThreadContext());
  if (m_behaviors.length() && !m_pCurrent) {
    // we are in the time window where behaviors are
    // queued but one just finished and NULLed itself
    // so we wait until the next behavior starts
    // this ensures we end with proper status if we abort
    while (!m_pCurrent) {
      aIO_MSSleep(m_ioRef, 1, NULL);
    }
  }
  m_pcVMM[aROBOT_APITEA_PRIMPROCID]->killProcess();

} // acpRobotInternal flushQueuedBehaviors method



/////////////////////////////////////////////////////////////////////
// acpRobotInternal flushQueue method
//
// only called from a dying behavior

void acpRobotInternal::flushQueue()
{
  aAssert(m_pcThread->isThreadContext());

  while (m_behaviors.head()) {
    acpRobotBehavior* pDangler;
    pDangler = m_behaviors.removeHead();
    pDangler->finish(aROBOT_ERRFLAG_WONTEXECUTE, NULL);
  }
  
} // acpRobotInternal flushQueue method



/////////////////////////////////////////////////////////////////////

void acpRobotInternal::logMessage(
  const int nlink,
  const int ncode,
  const char* text
)
{
  char sNum[8];
  char sMessage[aFILE_NAMEMAXCHARS + 16];
  
  aStringFromInt(sNum, nlink);
  aStringCopy(sMessage, "Link[");
  aStringCat(sMessage, sNum);
  aStringCat(sMessage, "], err=");

  aStringFromInt(sNum, (int)ncode);
  aStringCat(sMessage, sNum);
  aStringCat(sMessage, " : ");
  aStringCat(sMessage, text);

  if (m_errorStream)
    aStream_WriteLine(m_ioRef, m_errorStream, sMessage, NULL);
}



/////////////////////////////////////////////////////////////////////

acpRobotPackage* acpRobotInternal::readPackage(
  const char* pFileName
)
{
  aErr err = aErrNone;
  aStreamRef packageStream;
  acpRobotPackage* pPackage = NULL;
  
  aStream_CreateFileInput(m_ioRef, 
  			  pFileName, 
  			  aFileAreaObject,
  			  &packageStream, 
  			  &err);

  if (err == aErrNone) {
    pPackage = new acpRobotPackage(m_ioRef, packageStream);
    pPackage->initialize();
    aStream_Destroy(m_ioRef, packageStream, &err);
  }

  return pPackage;
}



/////////////////////////////////////////////////////////////////////

void acpRobotInternal::createRobotObject(
  acpPackageTag* pTag
)
{
  // create one of our acpRobot objects
  acpObject* pObj = NULL;
  acpProperty* pProp = NULL;
  switch (pTag->getTagType()) {
    case aTAG_apiPACKAGE:
    {
      // this will be the first object in the package
      // use it to get any global data about the package
      acpTag_apiPACKAGE* pPkgData = (acpTag_apiPACKAGE*)pTag;
      setDistanceUnitType(pPkgData->m_distanceUnit);
      setAngleUnitType(pPkgData->m_angleUnit);
      setMassUnitType(pPkgData->m_massUnit);
      break;
    }
    case aTAG_PROPERTY:
      pProp = new acpRobotUserProperty(this, (acpTag_PROPERTY*)pTag);
      break;
    case aTAG_PRIMITIVE:
      pObj = new acpRobotPrimitive(this, (acpTag_PRIMITIVE*)pTag);
      break;
    case aTAG_TEMPLATE:
      pObj = new acpRobotBaseObject(this, (acpTag_TEMPLATE*)pTag);
      break;
    case aTAG_USEROBJECT:
      pObj = new acpRobotUserObject(this, (acpTag_USEROBJECT*)pTag);
      break;
  }

  // link new object to the package tag
  // add subojects owned by main object
  if (pObj) {
    pTag->setRef(pObj);
    if (pTag->getOwnerID() == 0)
      addSubObject(pObj);
  } else if (pProp) {
    pTag->setRef(pProp);
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotInternal::assignLinkID(
  acpString& rName,
  int* pnLinkID
)
{
  int k = lookupString(m_listLinksUsed, rName);
  if (k == aROBOT_APITEA_NOTFOUND) {
    // default stream name is ""
    // link id will be zero-based
    acpString* pUsedLink = new acpString(rName);
    *pnLinkID = m_listLinksUsed.length();
    m_listLinksUsed.add(pUsedLink);
  } else {
    *pnLinkID = k;
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotInternal::collectTagsAndLinks(
  acpRobotPackage* pPackage
)
{
  acpPackageTag* pTag;

  // slurp in the package and put it in
  // a list that can be traversed multiple times
  // (it will take several passes to process it)
  while ((pTag = pPackage->consumeHeadTag()))
    m_tagList.add(pTag);

  // create objects used by the DLL
  // and set reference link in package tag
  // and add them to the main robot object
  listPkgTagsIter tags1(m_tagList);
  while ((pTag = tags1.next()))
    createRobotObject(pTag);

  // determine which streams are actually used
  // there is no need to create streams that are not used
  listPkgTagsIter tags2(m_tagList);
  while ((pTag = tags2.next())) {
    switch (pTag->getTagType()) {
      case aTAG_PROPERTY:
      {
        acpTag_PROPERTY* pPropTag = (acpTag_PROPERTY*)pTag;
        if (pPropTag->getOwnerID() == 0) {
          // only root-level properties can have a link name
          acpRobotUserProperty* pProp =
            (acpRobotUserProperty*)pTag->getRef();
          assignLinkID(pPropTag->m_linkname, &pProp->m_nLinkID);
        }
        break;
      }
      case aTAG_PRIMITIVE:
      {
        acpTag_PRIMITIVE* pPrimTag = (acpTag_PRIMITIVE*)pTag;
        acpRobotPrimitive* pPrim =
          (acpRobotPrimitive*)pTag->getRef();
        assignLinkID(pPrimTag->m_linkname, &pPrim->m_nLinkID);
        break;
      }
      case aTAG_USEROBJECT:
      {
        acpTag_USEROBJECT* pUObjTag = (acpTag_USEROBJECT*)pTag;
        acpRobotUserObject* pUObj =
          (acpRobotUserObject*)pTag->getRef();
        assignLinkID(pUObjTag->m_linkname, &pUObj->m_nLinkID);
        break;
      }
    } // switch
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotInternal::initTagData()
{
  acpPackageTag* pTag;

  // link SET and GET objects to parents
  listPkgTagsIter tags1(m_tagList);
  while ((pTag = tags1.next())) {
    switch (pTag->getTagType()) {
      case aTAG_SET:
      case aTAG_GET:
      {
        acpPackageTag* pParentTag = 
          m_tagList[pTag->getOwnerID() - 1];
        acpRobotUserProperty* pProp =
          (acpRobotUserProperty*)pParentTag->getRef();
        pProp->addTagData(pTag);
        break;
      }
    }
  }

  // properties now have their SET and GET data filled in
  // so now it's time to link objects to templates
  // and link child properties to parents
  acpValue v;
  listPkgTagsIter tags2(m_tagList);
  while ((pTag = tags2.next())) {
    switch (pTag->getTagType()) {
      case aTAG_PROPERTY:
      {
        acpTag_PROPERTY* pPropTag = (acpTag_PROPERTY*)pTag;
        acpRobotUserProperty* pProp =
          (acpRobotUserProperty*)pTag->getRef();
        if (pTag->getOwnerID() == 0) {
          // global property
          initValueFromString(pPropTag->m_default, pPropTag->m_flags, &v);
          addProperty(pProp, &v);
        }  else {
          acpPackageTag* pParentTag = 
            m_tagList[pTag->getOwnerID() - 1];
          acpObject* pObj = (acpObject*)pParentTag->getRef();
          initValueFromString(pPropTag->m_default, pPropTag->m_flags, &v);
          if (pParentTag->getTagType() == aTAG_PRIMITIVE) {
            acpRobotPrimitive* pPrim = (acpRobotPrimitive*)pObj;
            pPrim->addPrimitiveProperty(pProp, &v);
          }
          if (pParentTag->getTagType() == aTAG_TEMPLATE) {
            acpRobotBaseObject* pTemplate = (acpRobotBaseObject*)pObj;
            pTemplate->addBaseProperty(pProp, &v);
          }
        }
        break;
      }
      case aTAG_USEROBJECT:
      {
        acpTag_USEROBJECT* pObjTag = (acpTag_USEROBJECT*)pTag;
        acpRobotBaseObject* pTemplate =
          (acpRobotBaseObject*)getNamedSubObject("template", pObjTag->m_templatename);
        acpRobotUserObject* pObj = (acpRobotUserObject*)pObjTag->getRef();
        pObj->setTemplate(pTemplate);
        break;
      }
    } // switch
  }

  // last step is to apply initializers
  // for user object storage buffers
  listPkgTagsIter tags3(m_tagList);
  while ((pTag = tags3.next())) {
    if (pTag->getTagType() == aTAG_INITIALIZER) {
      acpTag_INITIALIZER* pInitTag = (acpTag_INITIALIZER*)pTag;
      acpRobotUserObject* pObj =
        (acpRobotUserObject*)m_tagList[pTag->getOwnerID() - 1]->getRef();
      acpRobotBaseObject* pTemplate = pObj->getTemplate(); 
      int k = pTemplate->getPropertyIndex(pInitTag->m_name);
      aPROPERTY_FLAGS flags = pTemplate->getPropertyFlags(k);
      initValueFromString(pInitTag->m_svalue, flags, &v);
      pObj->setNamedValue(pInitTag->m_name, &v);
    }
  }
}


/////////////////////////////////////////////////////////////////////
void acpRobotInternal::initValueFromString(
  const char* pValStr,
  const aPROPERTY_FLAGS flags,
  acpValue* pVal)
{
  if (flags & aPROPERTY_FLAG_INT) {
    int n = 0;
    aIntFromString(&n, pValStr);
    pVal->set((int)n);
  } else if (flags & aPROPERTY_FLAG_BOOL) {
    int n = 0;
    aIntFromString(&n, pValStr);
    pVal->set((bool)n);
  } else if (flags & aPROPERTY_FLAG_FLOAT) {
    float f = 0.0f;
    aUtil_ParseFloat(&f, pValStr, NULL);
    pVal->set((float)f);
  }
}



/////////////////////////////////////////////////////////////////////
float acpRobotInternal::convertToGlobalUnits(
  const float f,
  const char cUnitType,
  const char cUnitCode)
{
  float newf = f;
  int nGlobalUnits = aROBOT_UNITS_NONE;

  switch(cUnitType) {
    case aROBOT_UNITS_DISTANCE:
      nGlobalUnits = distanceUnitType();
      break;
    case aROBOT_UNITS_ANGLE:
      nGlobalUnits = angleUnitType();
      break;
    case aROBOT_UNITS_MASS:
      nGlobalUnits = massUnitType();
      break;
  }
  
  if (cUnitCode != nGlobalUnits)
    newf = unitConvFac[(int)cUnitCode][(int)nGlobalUnits] * f;

  return newf;
}


/////////////////////////////////////////////////////////////////////
float acpRobotInternal::convertFromGlobalUnits(
  const float f,
  const char cUnitType,
  const char cUnitCode)
{
  float newf = f;
  int nGlobalUnits = aROBOT_UNITS_NONE;

  switch(cUnitType) {
    case aROBOT_UNITS_DISTANCE:
      nGlobalUnits = distanceUnitType();
      break;
    case aROBOT_UNITS_ANGLE:
      nGlobalUnits = angleUnitType();
      break;
    case aROBOT_UNITS_MASS:
      nGlobalUnits = massUnitType();
      break;
  }
  
  if (cUnitCode != nGlobalUnits)
    newf = unitConvFac[nGlobalUnits][(int)cUnitCode] * f;

  return newf;
}



/////////////////////////////////////////////////////////////////////
void acpRobotInternal::initConvFac()
{
  // one-to-one for anything undefined
  for (int i = 0; i < aROBOT_UNITS_CODES; i++)
    for (int j = 0; j < aROBOT_UNITS_CODES; j++)
      unitConvFac[i][j] = 1.0f;

  // otherwise add an entry in conversion matrix
  // (this requires a unique code for every unit)
  unitConvFac[aROBOT_UNITS_METERS][aROBOT_UNITS_FEET] = 3.2808f;
  unitConvFac[aROBOT_UNITS_METERS][aROBOT_UNITS_INCHES] = 39.3696f;
  unitConvFac[aROBOT_UNITS_INCHES][aROBOT_UNITS_METERS] = 0.0254f;
  unitConvFac[aROBOT_UNITS_INCHES][aROBOT_UNITS_FEET] = 0.0833333f;
  unitConvFac[aROBOT_UNITS_FEET][aROBOT_UNITS_METERS] = 0.3048f;
  unitConvFac[aROBOT_UNITS_FEET][aROBOT_UNITS_INCHES] = 12.0f;
  unitConvFac[aROBOT_UNITS_DEGREES][aROBOT_UNITS_RADIANS] = 0.01745f;
  unitConvFac[aROBOT_UNITS_RADIANS][aROBOT_UNITS_DEGREES] = 57.30f;
  unitConvFac[aROBOT_UNITS_KILOGRAMS][aROBOT_UNITS_POUNDS] = 2.2046f;
  unitConvFac[aROBOT_UNITS_POUNDS][aROBOT_UNITS_KILOGRAMS] = 0.4536f;
}



/////////////////////////////////////////////////////////////////////

acpStreamWriteLineMessage::acpStreamWriteLineMessage (
  aStreamRef stream,
  const char* pLine) :
  m_stream(stream)
{
  unsigned int len = aStringLen(pLine);
  m_pText = (char*)aMemAlloc(sizeof(char) * (len + 1));
  if (m_pText)
    aStringCopy(m_pText, pLine);
}

/////////////////////////////////////////////////////////////////////

void acpStreamWriteLineMessage::process()
{
  if (m_pText)
    aStream_WriteLine(aStreamLibRef(m_stream), m_stream, 
                      m_pText, NULL);
}
