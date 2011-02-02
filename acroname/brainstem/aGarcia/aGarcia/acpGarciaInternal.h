/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaInternal.h                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API library object.       //
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

#ifndef _acpGaricaInternal_H_
#define _acpGaricaInternal_H_

#include "aIO.h"
#include "aUI.h"
#include "aStem.h"

#include "acpObject.h"
#include "acpProperty.h"
#include "aGarciaProperties.h"
#include "acpRunable.h"
#include "acpMutex.h"
#include "acpThread.h"
#include "acpList.h"
#include "aGarciaOSExport.h"
#include "acpGarciaOSFactory.h"
#include "acpGarciaPrimitive.h"
#include "acpHTMLSite.h"
#include "acpBehavior.h"
#include "acpCallback.h"

#define aGARCIA_YIELDMS  		10
#define aGARCIA_ACTIVEHBCOUNT		2
#define aGARCIA_MAXSETTINGLEN		32
#define aGARCIA_SETTINGSFILE		"garcia.config"
#define aGARCIA_HBINTKEY		"hb-interval"
#define aGARCIA_HBINTDEFAULT		400
#define aGARCIA_DISTUNITKEY		"distance-units"
#define aGARCIA_TEXT_METERS		"meters"
#define aGARCIA_TEXT_FEET		"feet"
#define aGARCIA_TEXT_INCHES		"inches"
#define aGARCIA_ANGLEUNITKEY		"angle-units"
#define aGARCIA_TEXT_RADIANS		"radians"
#define aGARCIA_TEXT_DEGREES		"degrees"
#define aGARCIA_CAMERABOOMKEY		"camera-boom"
#define aGARCIA_SRF08COUNTKEY		"srf08-count"
#define aGARCIA_TEXT_YES		"yes"
#define aGARCIA_TEXT_NO			"no"

#define aGARCIA_PADIO_TIMEOUTMS 250
#define aGARCIA_A2DIO_TIMEOUTMS 250

class acpPacketMessage;
class acpGarciaMessage;
class acpGarcia;
class acpBehaviorList;

class acpGarciaInternal :
  public acpRunable, public acpObject
{
  public:
  
  				acpGarciaInternal(
  				  acpGarcia* pcGarcia);
    virtual			~acpGarciaInternal();

    aErr			writeToStream(const aStreamRef stream) const
    				  { return aErrNone; }
  
    acpBehavior*		createNamedBehavior (
    				  const char* pPrimitiveName,
    				  const char* pBehaviorName,
    				  acpBehaviorList* pParent = NULL);

    acpBehavior*		createBehavior (
    				  const int nPrimitiveIndex,
    				  const char* pBehaviorName,
    				  acpBehaviorList* pParent = NULL);

    void			queueBehavior (
    				  acpBehavior* pBehavior);
    void			queueBehaviorHead (
    				  acpBehavior* pBehavior);

    void			performBehavior (
    				  acpBehavior* pBehavior);

    void			flushQueuedBehaviors();
    void			flushQueue();


    bool			isActive()
    				  { return m_bActive; }
    				  
    acpGarcia*			getOpaque()
    				  { return m_pcGarcia; }

    bool			isIdle();

    unsigned int		ticksPerUnitDistance();
    unsigned int		ticksPerUnitRotation();
    const int			distanceUnitType()
    				  { return m_nDistanceUnitType; }
    const int			angleUnitType()
    				  { return m_nAngleUnitType; }
    void			setDistanceUnitType(const int n)
    				  { m_nDistanceUnitType = n; }
    void			setAngleUnitType(const int n)
    				  { m_nAngleUnitType = n; }

    void			addCallerMessage (
    				  acpMessage* pMessage);

    int				handleCallerMessages(
    				  const unsigned long nMSYield);

    void			goToSleep(
    				  const unsigned long nMSSleep);

    void			clearCurrent(acpBehavior* pDead = NULL);

    aStemLib			m_stemRef;
    aIOLib			m_ioRef;

    bool			hasStatus()
                                  { return m_statusStream != NULL; }
    void			addStatusLine(const char* pLine);
    char*			statusString(
    				  const short nStatus,
    				  char* pText,
    				  unsigned int nMaxChars);

    void			abortCurrentBehavior(
                                  const int nValue);

    bool			isThread()
    				  { return m_pcThread->isThreadContext(); }

  protected:
    aUILib			m_uiRef;
    aSettingFileRef		m_settings;
    
    aStreamRef			m_linkStream;

    acpHTMLSite*		m_pcViewSite;

    acpGarciaOSFactory*		m_pcFactory;

    acpThread*			m_pcThread;    
    int				run();

 private:
    bool			m_bHB;
    aStreamRef			m_statusStream;
    aStreamRef			m_errorStream;
    aStreamRef			m_relayStreams[128];
    int				m_nRelayIndex;
    int				m_nRelayStatus;

    acpGarcia*			m_pcGarcia;
    int				m_nDistanceUnitType;
    int				m_nAngleUnitType;
    
    acpMutex*			m_pcBehaviorMutex;
    acpList<acpBehavior>	m_behaviors;
    acpBehavior*		m_pCurrent;

    void			handlePacketOut (
    				  acpPacketMessage* pMessage);
    char			getPadValue (
    				  const unsigned char module,
    				  const unsigned char offset);
    void			getRangerValue (
    				  const unsigned char module,
    				  const unsigned char index,
    				  acpValue* pValue);
    float			getOdometerValue (
    				  const unsigned char module,
    				  const unsigned char index);
    short			getCounterValue (
    				  const unsigned char module,
    				  const unsigned char index);
    float			getAnalogValue (
    				  const unsigned char module,
    				  const unsigned char index);
    int				getDigitalValue (
    				  const unsigned char module,
    				  const unsigned char index);
    float			getServoPositionValue (
    				  const unsigned char module,
    				  const unsigned char index);
    unsigned char		getServoConfigValue (
    				  const unsigned char module,
    				  const unsigned char index);
    short			getServoLimitsValue (
    				  const unsigned char module,
    				  const unsigned char index);
    int				getSonarRangeValue (
				  const unsigned char module,
				  const unsigned char addr,
				  const unsigned char units,
				  const int nvals,
				  short* psBuff);
    int				getSonarLightValue (
				  const unsigned char module,
				  const unsigned char addr);
    int				getUserObjValue (
				  const unsigned char module,
				  const unsigned char size,
				  const unsigned char replybyte,
				  const unsigned long timeout,
				  char* data);

    static aErr 		sHBProc (
    				  const aBool bHBOn,
				  void* ref);
    void			finishBehavior (
    				  const short status);

    void			sendStemPacket (
  				  unsigned char address,
				  unsigned char length,
				  char* data);

    void			dispatchPacketData (
  				  unsigned char address,
				  unsigned char length,
				  char* data);
    
    bool			m_bActive;
    bool			m_bSleeping;
    int				m_nHBCount;
    unsigned long		m_nLastHB;
    unsigned long		m_nHBInterval;
    unsigned long		m_nWakeUpTime;
    
    int				m_nNextBehaviorID;
    
    acpCallback*		m_pcHBCallback;

    acpMutex*			m_pcCallerMessageMutex;
    acpList<acpMessage>		m_callerMessages;

  friend class acpGarciaPrimitive;
  friend class acpPacketMessage;
  friend class acpGetPadMessage;
  friend class acpGetRangerValueMessage;
  friend class acpGetOdometerMessage;
  friend class acpGetCounterMessage;
  friend class acpGetAnalogMessage;
  friend class acpGetDigitalMessage;
  friend class acpGetServoPositionMessage;
  friend class acpGetServoConfigMessage;
  friend class acpGetServoLimitsMessage;
  friend class acpGetSonarRangeMessage;
  friend class acpGetSonarLightMessage;
  friend class acpGetUserObjMessage;
  friend class acpGarciaHTMLMain;
  friend class acpGarciaHTMLPrimitives;
  friend class acpCallbackMessage;
  friend class acpBehaviorList;
  friend class acpGarciaXMLScript;
  friend class acpGarciaMessage;
  friend class acpGarciaServo;
  friend class acpGarciaCamera;

  friend class acpGarciaProperty;
  friend class acpGarciaHeartbeatCallbackProperty;
  friend class acpGarciaHeartbeatStatusProperty;
  friend class acpGarciaStatusStreamProperty;
  friend class acpGarciaErrorStreamProperty;
  friend class acpGarciaRelayIndexProperty;
  friend class acpGarciaRelayStatusProperty;
  friend class acpGarciaRelayByteProperty;

  friend class acpGarciaServoProperty;
  friend class acpGarciaSonarProperty;
  friend class acpGarciaCameraProperty;
  friend class acpGarciaUserObjProperty;
};


/////////////////////////////////////////////////////////////////////

class acpGarciaMessage :
  public acpMessage
{
  public:
                                acpGarciaMessage(
                                  acpGarcia* pcGarcia,
                                  acpCallback* pcCallback) :
                                  m_pcGarcia(pcGarcia),
                                  m_pcCallback(pcCallback) {}

   void	                        process()
                                  { m_pcCallback->call(); }

  private:
    acpGarcia*		        m_pcGarcia;
    acpCallback* 		m_pcCallback;
};


/////////////////////////////////////////////////////////////////////

class acpStreamWriteLineMessage :
  public acpMessage
{
  public:
                                acpStreamWriteLineMessage(
                                  aStreamRef stream,
                                  const char* pLine);
   virtual                      ~acpStreamWriteLineMessage()
                                  { if (m_pText) aMemFree(m_pText); }

   void	                        process();

  private:
    aStreamRef			m_stream;
    char*			m_pText;
};


#endif // _acpGaricaInternal_H_
