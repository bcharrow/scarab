/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaInternal.cpp                                     //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API library object.   //
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
#include "aGarciaDefs.tea"

#include "aUtil.h"
#include "aStreamUtil.h"
#include "aGarciaGeom.h"

#include "acpException.h"
#include "acpGarciaInternal.h"

#include "acpPacketMessage.h"
#include "acpGetPadMessage.h"
#include "acpGetRangerValueMessage.h"
#include "acpGetOdometerMessage.h"
#include "acpGetCounterMessage.h"
#include "acpGetAnalogMessage.h"
#include "acpGetDigitalMessage.h"
#include "acpGetServoMessage.h"
#include "acpGetSonarMessage.h"
#include "acpGetUserObjMessage.h"
#include "acpGarciaHTMLMain.h"

#include "acpGarciaProperties.h"
#include "acpGarciaServo.h"
#include "acpGarciaSonar.h"
#include "acpGarciaCamera.h"
#include "acpGarciaUserObj.h"

#include "acpGarciaTurn.h"
#include "acpGarciaMove.h"
#include "acpGarciaPivot.h"
#include "acpGarciaHug.h"
#include "acpGarciaAlign.h"
#include "acpGarciaNull.h"
#include "acpGarciaDock.h"
#include "acpGarciaSay.h"
#include "acpGarciaGlobal.h"
#include "acpGarciaSleep.h"
#include "acpGarciaXMLScript.h"

#include "acpBehaviorList.h"

#if defined(aWIN) || defined(aWINCE)
#include "win32_acpGarciaOSFactory.h"
#endif // aWIN
#ifdef aUNIX
#include "unix_acpGarciaOSFactory.h"
#endif // aUNIX





/////////////////////////////////////////////////////////////////////
// acpGarciaInternal constructor
//

acpGarciaInternal::acpGarciaInternal (
  acpGarcia* pcGarcia
) :
  acpObject("garcia", "garcia"),
  m_ioRef(NULL),
  m_uiRef(NULL),
  m_settings(NULL),
  m_linkStream(NULL),
  m_pcFactory(NULL),
  m_pcThread(NULL),
  m_bHB(false),
  m_statusStream(NULL),
  m_errorStream(NULL),
  m_nRelayIndex(-1),
  m_nRelayStatus(aErrNone),
  m_pcGarcia(pcGarcia),
  m_nDistanceUnitType(aGARCIA_DISTANCE_METERS),
  m_nAngleUnitType(aGARCIA_ANGLE_RADIANS),
  m_pCurrent(NULL),
  m_bActive(false),
  m_bSleeping(false),
  m_nHBCount(0),
  m_nLastHB(0L),
  m_nWakeUpTime(0L),
  m_nNextBehaviorID(100),
  m_pcHBCallback(NULL)//,
//  m_nNumPrimitives(0)  
{
  aErr err;
  char* pcSetting;

  // get the aIO library reference
  aIO_GetLibRef(&m_ioRef, &err);
  aAssert(err == aErrNone);

  // get settings if non were provided
  if (!m_settings) {
    aSettingFile_Create(m_ioRef, 
    			    aGARCIA_MAXSETTINGLEN,
    			    aGARCIA_SETTINGSFILE,
    			    &m_settings, &err);
    aAssert(err == aErrNone);
  }

  // get the aUI library reference
  aUI_GetLibRef(&m_uiRef, &err);
  aAssert(err == aErrNone);

  // get the aStem library reference
  aStem_GetLibRef(&m_stemRef, &err);
  aAssert(err == aErrNone);

  // build stream for incoming serial relay packets
  aBZero(m_relayStreams, 128 * sizeof (aStreamRef));

  // ensure a default baudrate of 38400
  int baudrate;
  if (err == aErrNone)
    aSettingFile_GetInt(m_ioRef, 
    			m_settings, 
    			BAUDRATEKEY, 
    			&baudrate,
    			22,
    			&err);
  if ((err == aErrNone) && (baudrate == 22)) {
    aSettingFile_SetKey(m_ioRef, 
    			m_settings,
    			BAUDRATEKEY,
    			"38400", &err);
  }  
  
  // set up the garcia library stream
  aStreamUtil_CreateSettingStream(m_ioRef, 
  				  m_uiRef,
  				  m_stemRef,
  				  m_settings,
  				  &m_linkStream,
  				  &err);
  aAssert(err == aErrNone);

  // set up the heartbeat callback
  aStem_SetHBCallback(m_stemRef, sHBProc, this, &err);
  aAssert(err == aErrNone);

  // build the debugging http server with its pages
  m_pcViewSite = new acpHTMLSite(m_ioRef, m_uiRef, m_settings);
  m_pcViewSite->addPage("/apiview", 
  			new acpGarciaHTMLMain(this));
  m_pcViewSite->addPage("/apiprimitives", 
  			new acpGarciaHTMLPrimitives(this));
  m_pcViewSite->addPage("/apiprimitiveparams", 
  			new acpGarciaHTMLPrimitiveParams(this));
  m_pcViewSite->addPage("/apiaddbehavior", 
  			new acpGarciaHTMLAddBehavior(this));

  // build the OS factory, this should be the only OS specific
  // code in the API.
#if defined(aWIN) || defined(aWINCE)
  m_pcFactory = new win32_acpGarciaOSFactory();
#endif /* aWIN */
#ifdef aUNIX
  m_pcFactory = new unix_acpGarciaOSFactory();
#endif /* aUNIX */    
  aAssert(m_pcFactory);

  // global properties
  addProperty(new acpGarciaActiveProperty(this));
  addProperty(new acpGarciaIdleProperty(this));
  addProperty(new acpGarciaUserLEDProperty(this));
  addProperty(new acpGarciaUserButtonProperty(this));
  addProperty(new acpGarciaBatteryLevelProperty(this));
  addProperty(new acpGarciaBatteryVoltageProperty(this));
  addProperty(new acpGarciaDistanceUnitsProperty(this));
  addProperty(new acpGarciaDistanceUnitsStringProperty(this));
  addProperty(new acpGarciaAngleUnitsProperty(this));
  addProperty(new acpGarciaAngleUnitsStringProperty(this));
  addProperty(new acpGarciaIRTransmitProperty(this));
  addProperty(new acpGarciaIRReceiveProperty(this));
  addProperty(new acpGarciaUserFlagsProperty(this));
  addProperty(new acpGarciaStatusProperty(this));
  addProperty(new acpGarciaStallThresholdProperty(this));
  addProperty(new acpGarciaStallQueueSizeProperty(this));
  addProperty(new acpGarciaSpeedProperty(this));
  addProperty(new acpGarciaStatusStreamProperty(this));
  addProperty(new acpGarciaErrorStreamProperty(this));
  addProperty(new acpGarciaRelayIndexProperty(this));
  addProperty(new acpGarciaRelayStatusProperty(this));
  addProperty(new acpGarciaRelayByteProperty(this));

  // ranger properties
  addProperty(new acpGarciaFrontRangerLeftProperty(this));
  addProperty(new acpGarciaFrontRangerRightProperty(this));
  addProperty(new acpGarciaSideRangerLeftProperty(this));
  addProperty(new acpGarciaSideRangerRightProperty(this));
  addProperty(new acpGarciaRearRangerLeftProperty(this));
  addProperty(new acpGarciaRearRangerRightProperty(this));
  addProperty(new acpGarciaDownRangerLeftProperty(this));
  addProperty(new acpGarciaDownRangerRightProperty(this));
  addProperty(new acpGarciaFrontRangerThresholdProperty(this));
  addProperty(new acpGarciaSideRangerThresholdProperty(this));
  addProperty(new acpGarciaRearRangerThresholdProperty(this));
  addProperty(new acpGarciaFrontRangerEnableProperty(this));
  addProperty(new acpGarciaDownRangerEnableProperty(this));
  addProperty(new acpGarciaSideRangerEnableProperty(this));
  addProperty(new acpGarciaRearRangerEnableProperty(this));

  // motor properties
  addProperty(new acpGarciaDampedSpeedLeftProperty(this));
  addProperty(new acpGarciaDampedSpeedRightProperty(this));
  addProperty(new acpGarciaDistanceLeftProperty(this));
  addProperty(new acpGarciaDistanceRightProperty(this));

  // heartbeat properties
  addProperty(new acpGarciaHeartbeatCallbackProperty(this));
  addProperty(new acpGarciaHeartbeatStatusProperty(this));

  // check for camera (affects creation of servo objects)
  int servoUsedFlags = 0;
  bool bCameraBoom = 0;
  aSettingFile_GetString(m_ioRef, m_settings,
  		            aGARCIA_CAMERABOOMKEY,
  		            &pcSetting, aGARCIA_TEXT_NO,
  		            &err);
  aAssert(err == aErrNone);
  if (err == aErrNone) {
    if (!aStringCompare(pcSetting, aGARCIA_TEXT_YES)) {
      servoUsedFlags =
        (1 << aGCP_PAN_INDEX) | (1 << aGCP_TILT_INDEX);
      bCameraBoom = true;
    }
  }

  // check for sonars (affects creation of sonar objects)
  int nSonars = 0;
  if (err == aErrNone)
    aSettingFile_GetInt(m_ioRef, m_settings,
  		            aGARCIA_SRF08COUNTKEY,
  		            &nSonars, 0,
  		            &err);
  if (nSonars > 16) nSonars = 16;
  if (nSonars < 0) nSonars = 0;

  // add in all default subobjects (primitives)
  // (should this be done through a plugin mechanism???)
  addSubObject(new acpGarciaTurn(this));
  addSubObject(new acpGarciaMove(this));
  addSubObject(new acpGarciaPivot(this));
  addSubObject(new acpGarciaHug(this));
  addSubObject(new acpGarciaAlign(this));
  addSubObject(new acpGarciaNull(this));
  addSubObject(new acpGarciaDock(this));
  addSubObject(new acpGarciaSay(this));
  addSubObject(new acpGarciaGlobal(this));
  addSubObject(new acpGarciaSleep(this));
  addSubObject(new acpGarciaXMLScript(this));

  // add generic user-definable object
  // (permits customized user IO interaction)
  addSubObject(new acpGarciaUserObj("userobj_0", this));

  // handle conditional addition of subobjects
  if (!(servoUsedFlags & 0x01))
    addSubObject(new acpGarciaServo(aGSP_SERVO_0, aGARCIA_GP_ADDR, 0, this));
  if (!(servoUsedFlags & 0x02))
    addSubObject(new acpGarciaServo(aGSP_SERVO_1, aGARCIA_GP_ADDR, 1, this));
  if (!(servoUsedFlags & 0x04))
    addSubObject(new acpGarciaServo(aGSP_SERVO_2, aGARCIA_GP_ADDR, 2, this));
  if (!(servoUsedFlags & 0x08))
    addSubObject(new acpGarciaServo(aGSP_SERVO_3, aGARCIA_GP_ADDR, 3, this));
  if (bCameraBoom)
    addSubObject(new acpGarciaCamera(aGCP_CAMERA_BOOM, aGARCIA_GP_ADDR, this));
  if (nSonars) {
    int k;
    unsigned char addr = 0xE0;
    char sname[20];
    char snum[20];
    for (k = 0; k < nSonars; k++) {
      aStringCopy(sname, aGRP_SONAR_PREFIX);
      aStringFromInt(snum, k);
      aStringCat(sname, snum);
      addSubObject(new acpGarciaSonar(sname, aGARCIA_GP_ADDR, addr, this));
      addr+=2;
    }
  }  
  
  // set up the ack interval
  aAssert(m_settings);
  aSettingFile_GetULong(m_ioRef, m_settings,
  		            aGARCIA_HBINTKEY,
  		            &m_nHBInterval, aGARCIA_HBINTDEFAULT,
  		            &err);
  aAssert(err == aErrNone);


  // select distance units
  aSettingFile_GetString(m_ioRef, m_settings,
  		            aGARCIA_DISTUNITKEY,
  		            &pcSetting, aGARCIA_TEXT_METERS,
  		            &err);
  aAssert(err == aErrNone);
  if (err == aErrNone) {
    acpValue distString(pcSetting);
    setNamedValue(aGARCIA_PROPNAME_DISTUNITSSTR, &distString);
  }
    
  // select angle units
  aSettingFile_GetString(m_ioRef, m_settings,
  		            aGARCIA_ANGLEUNITKEY,
  		            &pcSetting, aGARCIA_TEXT_RADIANS,
  		            &err);
  aAssert(err == aErrNone);
  if (err == aErrNone) {
    acpValue angleString(pcSetting);
    setNamedValue(aGARCIA_PROPNAME_DISTUNITSSTR, &angleString);
  }


  m_pcBehaviorMutex = 
  	m_pcFactory->buildMutex("Behavior List Mutex");

  m_pcCallerMessageMutex = 
  	m_pcFactory->buildMutex("Caller Message Mutex");

  m_pcThread = m_pcFactory->buildThread("Garcia API");
  m_pcThread->start(this);

} // acpGarciaInternal constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal destructor
//

acpGarciaInternal::~acpGarciaInternal ()
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

  for (int i = 0; i < 128; i++) {
    if (m_relayStreams[i])
      aStream_Destroy(m_ioRef, m_relayStreams[i], NULL);
  }

  if (m_settings)
    aSettingFile_Destroy(m_ioRef, m_settings, NULL);
  if (m_stemRef)
    aStem_ReleaseLibRef(m_stemRef, NULL);
  if (m_uiRef)
    aUI_ReleaseLibRef(m_uiRef, NULL);
  if (m_ioRef)
    aIO_ReleaseLibRef(m_ioRef, NULL);

} // acpGarciaInternal destructor




/////////////////////////////////////////////////////////////////////
// acpGarciaInternal run method
//
// This is only to be called by the garcia private thread.
//

int acpGarciaInternal::run()
{
  aAssert(m_pcThread->isThreadContext());

  while (!m_pcThread->isDone()) {
    bool bYield = true;

    // make sure the garcia link is up and active
    aErr err;
    unsigned long now;
    bool bWakeUpNow = false;
    aIO_GetMSTicks(m_ioRef, &now, &err);
    aAssert(err == aErrNone);
    
    // if we're sleeping check for wakey-wakey time
    if (m_bSleeping && (now > m_nWakeUpTime)) {
      bWakeUpNow = true;
    }

    // process any pending behaviors in the queue
    if (now > m_nLastHB + m_nHBInterval) {
      m_bActive = false;
      m_nHBCount = 0;
    } else if (m_nHBCount == aGARCIA_ACTIVEHBCOUNT) {
      m_bActive = true;
      

      bool bRan = false;
      if (!m_pCurrent) {
        m_pcBehaviorMutex->lock();
        m_pCurrent = m_behaviors.removeHead();
        m_pcBehaviorMutex->unlock();
        if (m_pCurrent) {
          m_pCurrent->execute();
          bRan = true;
        }
      }

      // a script will break all links to itself
      // and set current pointer to null
      if (bRan && m_pCurrent) {
        if (m_pCurrent->m_pPrimitive->isImmediate()) {
          finishBehavior(0);
        }
      }

      // if nothing executed check if we are
      // sleeping and it is time to wake up
      if (!bRan) {
        if (m_bSleeping && bWakeUpNow) {
          m_bSleeping = false;
          finishBehavior(0);
        }
      }
    }

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
    m_pcViewSite->slice(bChanged);
    if (bChanged)
      bYield = false;

    // check for a packet and handle it if there is one
    aPacketRef packet;
    if (!aStem_GetPacket(m_stemRef, 
    			 NULL,
    			 NULL,
    			 0,
    			 &packet, 
    			 NULL)) {
      unsigned char address;
      unsigned char length;
      char data[aSTEMMAXPACKETBYTES];
      if (!aPacket_GetData(m_stemRef, packet, 
      		           &address, &length, data, NULL))
        dispatchPacketData(address, length, data);

      aPacket_Destroy(m_stemRef, packet, NULL);

      // if we got a packet, don't yield the thread
      bYield = false;
    }

    // yield for a bit so we don't swamp the processor when nothing
    // is going on
    if (bYield)
      m_pcThread->yield(aGARCIA_YIELDMS);

  } // while 

  // Destroy all behaviors up front here so that we don't get calls
  // later in the destructor that rely on this thread (that is going
  // down here).
  while (m_behaviors.head()) {
    acpBehavior* pBehavior;
    pBehavior = m_behaviors.removeTail();
    delete pBehavior;
  }

  // kill any current behavior running on the robot to avoid any
  // callbacks
  if (m_pCurrent) {
    abortCurrentBehavior(aGARCIA_ERRFLAG_ABORT);
    // stall to ensure the serial bytes get out
    aIO_MSSleep(m_ioRef, 100, NULL);
  }

  return 0;

} // acpGarciaInternal run method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal addStatusLine method
//

void acpGarciaInternal::addStatusLine(const char* pLine)
{
  if (m_statusStream) {
    addCallerMessage(
    	new acpStreamWriteLineMessage(m_statusStream, pLine));
  }
} // acpGarciaInternal addStatusLine method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal statusString method
//

char* acpGarciaInternal::statusString (
  const short nStatus,
  char* pText,
  unsigned int nMaxChars
)
{
  aAssert(pText);
  *pText = '\0';
  
  switch (nStatus) {

  case aGARCIA_ERRFLAG_NORMAL:
    aStringNCopy(pText, "No Error", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_STALL:
    aStringNCopy(pText, "Motor Stall", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_FRONTR_LEFT:
    aStringNCopy(pText, "Front Left Ranger", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_FRONTR_RIGHT:
    aStringNCopy(pText, "Front Right Ranger", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_REARR_LEFT:
    aStringNCopy(pText, "Rear Left Ranger", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_REARR_RIGHT:
    aStringNCopy(pText, "Rear Right Ranger", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_SIDER_LEFT:
    aStringNCopy(pText, "Side Left Ranger", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_SIDER_RIGHT:
    aStringNCopy(pText, "Side Right Ranger", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_FALL_LEFT:
    aStringNCopy(pText, "Left Edge Detector", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_FALL_RIGHT:
    aStringNCopy(pText, "Right Edge Detector", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_ABORT:
    aStringNCopy(pText, "Aborted", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_NOTEXECUTED:
    aStringNCopy(pText, "Not Executed", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_WONTEXECUTE:
    aStringNCopy(pText, "Will Not Execute", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_BATT:
    aStringNCopy(pText, "Battery Low", nMaxChars);
    break;

  case aGARCIA_ERRFLAG_IRRX:
    aStringNCopy(pText, "IR Byte Received", nMaxChars);
    break;

  } // nStatus switch
  
  return pText;

} // acpGarciaInternal statusString method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal queueBehavior method
//
// This code can be called from any thread.

void acpGarciaInternal::queueBehavior(acpBehavior* pBehavior)
{ 
  aAssert(pBehavior);

  m_pcBehaviorMutex->lock();
  m_behaviors.addToTail(pBehavior);
  m_pcBehaviorMutex->unlock();

} // acpGarciaInternal queueBehavior



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal promoteBehaviorFront method
//
// This code can be called from any thread.

void acpGarciaInternal::queueBehaviorHead(acpBehavior* pBehavior)
{ 
  aAssert(pBehavior);

  m_pcBehaviorMutex->lock();
  m_behaviors.addToHead(pBehavior);
  m_pcBehaviorMutex->unlock();

} // acpGarciaInternal queueBehaviorFront



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal addCallerMessage method
//

void acpGarciaInternal::addCallerMessage (
  acpMessage* pMessage
)
{
  aAssert(pMessage);
  
  m_pcCallerMessageMutex->lock();
  m_callerMessages.addToTail(pMessage);
  m_pcCallerMessageMutex->unlock();

} // acpGarciaInternal::addCallerMessage method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal isIdle method
//

bool acpGarciaInternal::isIdle()
{
  return ((m_behaviors.length() == 0) && !m_pCurrent);

} // acpGarciaInternal isIdle



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal ticksPerUnitDistance method
//

unsigned int acpGarciaInternal::ticksPerUnitDistance()
{ 
  switch (m_nDistanceUnitType) {

    case aGARCIA_DISTANCE_METERS:
      return aGARCIA_TICKS_PER_METER;
      break;
    case aGARCIA_DISTANCE_FEET:
      return aGARCIA_TICKS_PER_FOOT;
      break;
    case aGARCIA_DISTANCE_INCHES:
      return (aGARCIA_TICKS_PER_FOOT / 12);
      break;

  } // switch
  
  return 0;

} // acpGarciaInternal ticksPerUnitDistance method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal ticksPerUnitRotation method
//

unsigned int acpGarciaInternal::ticksPerUnitRotation()
{ 
  switch (m_nAngleUnitType) {

    case aGARCIA_ANGLE_RADIANS:
      return aGARCIA_TICKS_PER_RADIAN;
      break;
    case aGARCIA_ANGLE_DEGREES:
      return aGARCIA_TICKS_PER_DEGREE;
      break;

  } // switch
  
  return 0;

} // acpGarciaInternal ticksPerUnitRotation method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal createBehavior method
//

acpBehavior* acpGarciaInternal::createNamedBehavior (
  const char* pPrimitiveName,
  const char* pBehaviorName,
  acpBehaviorList* pParent // = NULL
)
{
  acpGarciaPrimitive* pPrimitive =
    (acpGarciaPrimitive*)getNamedSubObject("primitive", pPrimitiveName);
  aAssert(pPrimitive);

  return pPrimitive->factoryBehavior(pBehaviorName, 
  				     pPrimitive, 
  			 	     m_nNextBehaviorID++, 
  			 	     this,
  			 	     pParent);

} // acpGarciaInternal createBehavior method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal createBehavior method
//

acpBehavior* acpGarciaInternal::createBehavior (
  const int nPrimitiveIndex,
  const char* pBehaviorName,
  acpBehaviorList* pParent // = NULL
)
{
  acpGarciaPrimitive* pPrimitive =
    (acpGarciaPrimitive*)getSubObject(nPrimitiveIndex);
  aAssert(pPrimitive);

  return pPrimitive->factoryBehavior(pBehaviorName, 
  				     pPrimitive, 
  			 	     m_nNextBehaviorID++, 
  			 	     this,
  			 	     pParent);

} // acpGarciaInternal createBehavior method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal handlePacketOut method
//
// Must be called from the thread context.
//

void acpGarciaInternal::handlePacketOut(acpPacketMessage* pMessage)
{
  aAssert(m_pcThread->isThreadContext());

  aErr err;
  aPacketRef packet;
  aPacket_Create(m_stemRef,
  		     pMessage->m_address,
  		     pMessage->m_length,
  		     pMessage->m_data,
  		     &packet, &err);
  aAssert(err == aErrNone);
  
  aStem_SendPacket(m_stemRef, packet, &err);
  aAssert(err == aErrNone);

} // acpGarciaInternal handlePacketOut method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getPadValue method
//

char acpGarciaInternal::getPadValue (
  const unsigned char module,
  const unsigned char offset
)
{
  char value = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetPadMessage* pMessage = 
  	new acpGetPadMessage(this, module, offset, &value);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return value;

} // acpGarciaInternal getPadValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getRangerValue method
//

void acpGarciaInternal::getRangerValue (
  const unsigned char id,
  const unsigned char code,
  acpValue* pValue
)
{
  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetRangerValueMessage* pMessage = 
  	new acpGetRangerValueMessage(this, id, code, pValue);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

} // acpGarciaInternal getRangerValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getOdometer method
//

float acpGarciaInternal::getOdometerValue (
  const unsigned char module,
  const unsigned char index
)
{
  float fValue = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetOdometerMessage* pMessage = 
  	new acpGetOdometerMessage(this, module, index, &fValue);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return fValue;

} // acpGarciaInternal getOdometer method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getCounterValue method
//

short acpGarciaInternal::getCounterValue (
  const unsigned char module,
  const unsigned char index
)
{
  short sValue;

  acpGetCounterMessage* pMessage = 
	new acpGetCounterMessage(this, module, index, &sValue);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return sValue;

} // acpGarciaInternal getCounterValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getAnalog method
//

float acpGarciaInternal::getAnalogValue (
  const unsigned char module,
  const unsigned char index
)
{
  float fValue = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetAnalogMessage* pMessage = 
  	new acpGetAnalogMessage(this, module, index, &fValue);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return fValue;

} // acpGarciaInternal getAnalog method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getDigital method
//

int acpGarciaInternal::getDigitalValue (
  const unsigned char module,
  const unsigned char index
)
{
  int nValue = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetDigitalMessage* pMessage = 
  	new acpGetDigitalMessage(this, module, index, &nValue);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return nValue;

} // acpGarciaInternal getDigital method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getServoPositionValue method
//

float acpGarciaInternal::getServoPositionValue (
  const unsigned char module,
  const unsigned char index
)
{
  float value = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetServoPositionMessage* pMessage = 
    new acpGetServoPositionMessage(this, module, index, &value);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return value;

} // acpGarciaInternal getServoPositionValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getServoConfigValue method
//

unsigned char acpGarciaInternal::getServoConfigValue (
  const unsigned char module,
  const unsigned char index
)
{
  unsigned char value = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetServoConfigMessage* pMessage = 
    new acpGetServoConfigMessage(this, module, index, &value);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return value;

} // acpGarciaInternal getServoConfigValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getServoLimitsValue method
//

short acpGarciaInternal::getServoLimitsValue (
  const unsigned char module,
  const unsigned char index
)
{
  short value = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetServoLimitsMessage* pMessage = 
    new acpGetServoLimitsMessage(this, module, index, &value);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return value;

} // acpGarciaInternal getServoLimitsValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getSonarRangeValue method
//

int acpGarciaInternal::getSonarRangeValue (
  const unsigned char module,
  const unsigned char addr,
  const unsigned char units,
  const int nvals,
  short* psBuff
)
{
  int value = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetSonarRangeMessage* pMessage = 
    new acpGetSonarRangeMessage(this, module, addr, units, nvals, psBuff, &value);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return value;

} // acpGarciaInternal getSonarRangeValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getSonarLightValue method
//

int acpGarciaInternal::getSonarLightValue (
  const unsigned char module,
  const unsigned char addr
)
{
  int value = 0;

  // This message keeps the pointer to value and when
  // the message is processed, it sends the request 
  // packet for the padIO and waits for the return,
  // setting value to the returned data byte.
  acpGetSonarLightMessage* pMessage = 
    new acpGetSonarLightMessage(this, module, addr, &value);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return value;

} // acpGarciaInternal getSonarLightValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal getUserObjValue method
//

int acpGarciaInternal::getUserObjValue (
  const unsigned char module,
  const unsigned char size,
  const unsigned char replybyte,
  const unsigned long timeout,
  char* data
)
{
  int err = aErrNone;
  
  // This message keeps all the data needed to
  // send a user command and get a reply.
  acpGetUserObjMessage* pMessage = 
    new acpGetUserObjMessage(this, module, size, replybyte, timeout, data, &err);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcThread->sendMessage(pMessage, false);

  return err;

} // acpGarciaInternal getServoLimitsValue method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal sHBProc method
//

aErr acpGarciaInternal::sHBProc (
  const aBool bHBOn,
  void* ref)
{
  acpGarciaInternal* pGarciaInternal = (acpGarciaInternal*)ref;

  aAssert(pGarciaInternal);
  
  // record the time of the heartbeat
  aErr err;
  aIO_GetMSTicks(pGarciaInternal->m_ioRef,
    		 &pGarciaInternal->m_nLastHB,
    		 &err);
  aAssert(err == aErrNone);

  // store the heartbeat state
  pGarciaInternal->m_bHB = (bHBOn == aTrue);

  // count sequential beats
  // (more than one beat may be necessary for true active condition)
  if (pGarciaInternal->m_nHBCount < aGARCIA_ACTIVEHBCOUNT)
    pGarciaInternal->m_nHBCount++;

  // now, message the callback if it is present
  if (pGarciaInternal->m_pcHBCallback) {
    pGarciaInternal->addCallerMessage(
    	new acpGarciaMessage(pGarciaInternal->m_pcGarcia,
                             pGarciaInternal->m_pcHBCallback));
  }

  return err;

} // acpGarciaInternal sHBProc routine



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal finishBehavior method
//
// can be called from anywhere
//

void acpGarciaInternal::finishBehavior(const short status)
{
  aAssert(m_pcThread->isThreadContext());
  aAssert(m_pCurrent);

  m_pCurrent->finish(status, NULL);

} // acpGarciaInternal finishBehavior



/////////////////////////////////////////////////////////////////////
// acpGarciaInternalLink dispatchPacketData method
//
// Must be called from the thread context.
//

void acpGarciaInternal::sendStemPacket (
  unsigned char address,
  unsigned char length,
  char* data
)
{
  acpPacketMessage* pMessage = 
  	new acpPacketMessage(this, address, length, data);

  // For efficiency, we only send the packet as a message if 
  // we are not in the serial thread
  if (!m_pcThread->isThreadContext()) {
    m_pcThread->sendMessage(pMessage, true);
  } else {
    handlePacketOut(pMessage);
    delete pMessage;
  }

} // acpGarciaInternal sendStemPacket method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal dispatchPacketData method
//
// Must be called from the thread context.
//

void acpGarciaInternal::dispatchPacketData (
  unsigned char address,
  unsigned char length,
  char* data
)
{
  aAssert(m_pcThread->isThreadContext());

  if (!length)
    return;
  
  unsigned char cmd = (unsigned char)data[0];
  unsigned char temp;

  switch (cmd) {
  
  case cmdMSG:
    aAssert(length > 1);
    temp = (unsigned char)data[1];

    switch (temp) {
      case msg_vmExit:
        // [0] cmdMSG
        // [1] msg_vmExit
        // [2] exit condition (0 is normal VM termination)
        // [3] process ID
        // [4] high byte of 2-byte status
        // [5] low byte of 2-byte status
        short status;
        if (length > 4) {
          aAssert(length > 4);
          status = aUtil_RetrieveShort(&data[4]);
        } else
          status = 0;
        finishBehavior(status);
        break;

    } // switch
    break;

  default:
#if 0
    acpPacketMessage* pPacket = 
      new acpPacketMessage(this, address, length, data);
    m_pMutex->lock();
    m_inbound.addToTail(pPacket);
    m_pMutex->unlock();
#endif
    break;

  } // switch 

} // dispatchPacketData


/////////////////////////////////////////////////////////////////////

void acpGarciaInternal::abortCurrentBehavior(
  const int nValue
)
{
  if (m_pCurrent) {
    char data[4];
    data[0] = cmdPAD_IO;
    data[1] = aGARCIA_MOTO_PADS_STATUS;
    aUtil_StoreShort(&data[2], (short)nValue);
    sendStemPacket(aGARCIA_MOTO_ADDR, 4, data);
  }
}


/////////////////////////////////////////////////////////////////////
// acpGarciaInternal handleCallerMessages method
//
// Can be called from any thread context.
//

int acpGarciaInternal::handleCallerMessages (
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

} // acpGarciaInternal handleCallerMessages method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal goToSleep method
//
// Can be called from any thread context.
//

void acpGarciaInternal::goToSleep (
  const unsigned long nMSSleep
)
{
  if (!m_bSleeping) {
    unsigned long now;
    aIO_GetMSTicks(m_ioRef, &now, NULL);
    m_nWakeUpTime = now + nMSSleep;
    m_bSleeping = true;
  }
} // acpGarciaInternal goToSleep method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal clearCurrent method

void acpGarciaInternal::clearCurrent (
  acpBehavior* pDead // = NULL
)
{
  if ((pDead == m_pCurrent) || !pDead) {
    m_pCurrent = NULL;
  }

} // acpGarciaInternal clearCurrent



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal flushQueuedBehaviors method
//
// Can be called from any thread context.
//
// Might want to handle calling the current behavior's callback
// with the aGARCIA_ERRFLAG_ABORT status

void acpGarciaInternal::flushQueuedBehaviors()
{
  m_pcBehaviorMutex->lock();
  if (m_pCurrent) {
    acpValue flag(aGARCIA_ERRFLAG_ABORT);
    m_pCurrent->setNamedValue(aGARCIA_PROPNAME_COMPSTATUS, &flag);

    // kill active primitive
    // (either sleeping or running TEA program)
    // sleep will never fail unless killed manually
    if (!m_bSleeping) {

// clear out the current behavior

      abortCurrentBehavior(aGARCIA_ERRFLAG_ABORT);

// equal to but more efficient than

//      acpValue val(aGARCIA_ERRFLAG_ABORT);
//      setNamedValue("status", &val);
    } else {
      unsigned long now;
      aIO_GetMSTicks(m_ioRef, &now, NULL);
      m_nWakeUpTime = now;
    }
  }
  m_pcBehaviorMutex->unlock();

} // acpGarciaInternal flushQueuedBehaviors method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal flushQueue method
//
// only called from a dying behavior

void acpGarciaInternal::flushQueue()
{
  aAssert(m_pcThread->isThreadContext());

  while (m_behaviors.head()) {
    acpBehavior* pDangler;
    pDangler = m_behaviors.removeHead();
    pDangler->finish(aGARCIA_ERRFLAG_WONTEXECUTE, NULL);
  }
  
  // we shouldn't explicitly clear the current pointer.  The 
  // destructor for a behavior handles this
  // m_pCurrent = NULL;

} // acpGarciaInternal flushQueue method



/////////////////////////////////////////////////////////////////////
// acpGarcia flushSerialRelayBuffer method
//
/*
aErr acpGarciaInternal::getSerialRelayByte(
  char* pc
)
{
  aErr recErr = aErrNone;

  aStream_Read(
    m_ioRef, 
    m_relayStreams[1],
    pc,
    1,
    &recErr);

  return recErr;

} // acpGarcia flushSerialRelayBuffer method
*/


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
