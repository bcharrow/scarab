/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotInternal.h                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Robot API library object.        //
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

#ifndef _acpRobotInternal_H_
#define _acpRobotInternal_H_

#include "aIO.h"
#include "aUI.h"
#include "aStem.h"
#include "aTEAvm.h"

#include "acpStem.h"
#include "acpObject.h"
#include "acpProperty.h"
#include "acpRunable.h"
#include "acpMutex.h"
#include "acpThread.h"
#include "acpList.h"
#include "acpHTMLSite.h"
#include "acpCallback.h"

#include "aRobotOSExport.h"
#include "acpRobotVMManager.h"
#include "acpRobotOSFactory.h"
#include "acpRobotBehavior.h"
#include "acpRobotPackage.h"

#define aROBOT_YIELDMS  		10
#define aROBOT_MAXSETTINGLEN		32
#define aROBOT_SETTINGSFILE		"robot.config"
#define aROBOT_HBINTKEY			"hb-interval"
#define aROBOT_HBINTDEFAULT		400
#define aROBOT_DISTUNITKEY		"distance-units"
#define aROBOT_TEXT_METERS		"meters"
#define aROBOT_TEXT_FEET		"feet"
#define aROBOT_TEXT_INCHES		"inches"
#define aROBOT_ANGLEUNITKEY		"angle-units"
#define aROBOT_TEXT_RADIANS		"radians"
#define aROBOT_TEXT_DEGREES		"degrees"
#define aROBOT_TEXT_YES			"yes"
#define aROBOT_TEXT_NO			"no"

typedef	acpList<acpPackageTag>		listPkgTags;
typedef	acpListIterator<acpPackageTag>	listPkgTagsIter;

class acpRobotPacketMessage;
class acpRobotMessage;
class acpRobot;
class acpRobotBehaviorList;
class acpRobotUserProperty;
class acpRobotVMManager;

class acpRobotInternal :
  public acpRunable, public acpObject
{
  public:
  
  				acpRobotInternal(
  				  acpRobot* pcRobot,
  				  const char* pName);
    virtual			~acpRobotInternal();

    aErr			writeToStream(const aStreamRef stream) const
    				  { return aErrNone; }
  
    acpRobotBehavior*		createNamedBehavior (
    				  const char* pPrimitiveName,
    				  const char* pBehaviorName,
    				  acpRobotBehaviorList* pParent = NULL);

    acpRobotBehavior*		createBehavior (
    				  const int nPrimitiveIndex,
    				  const char* pBehaviorName,
    				  acpRobotBehaviorList* pParent = NULL);

    void			queueBehavior (
    				  acpRobotBehavior* pBehavior);
    void			queueBehaviorHead (
    				  acpRobotBehavior* pBehavior);

    void			performBehavior (
    				  acpRobotBehavior* pBehavior);

    void			flushQueuedBehaviors();
    void			flushQueue();


    bool			isActive()
    				  { return m_bActive; }
    				  
    acpRobot*			getOpaque()
    				  { return m_pcRobot; }

    bool			isIdle();

    const int			distanceUnitType()
    				  { return m_nDistanceUnitType; }
    const int			angleUnitType()
    				  { return m_nAngleUnitType; }
    const int			massUnitType()
    				  { return m_nMassUnitType; }
    void			setDistanceUnitType(const int n)
    				  { m_nDistanceUnitType = n; }
    void			setAngleUnitType(const int n)
    				  { m_nAngleUnitType = n; }
    void			setMassUnitType(const int n)
    				  { m_nMassUnitType = n; }
    float			convertToGlobalUnits(
				  const float f,
				  const char cUnitType,
				  const char cUnitCode);
    float			convertFromGlobalUnits(
				  const float f,
				  const char cUnitType,
				  const char cUnitCode);

    void			addCallerMessage (
    				  acpMessage* pMessage);

    int				handleCallerMessages(
    				  const unsigned long nMSYield);

    void			goToSleep(
    				  const unsigned long nMSSleep);

    void			clearCurrent(acpRobotBehavior* pDead = NULL);

    aIOLib			m_ioRef;

    bool			hasStatus()
                                  { return m_statusStream != NULL; }
    void			addStatusLine(const char* pLine);
    char*			statusString(
    				  const short nStatus,
    				  char* pText,
    				  unsigned int nMaxChars);

    bool			isThread()
    				  { return m_pcThread->isThreadContext(); }

    void			logMessage(
				  const int nlink,
				  const int ncode,
				  const char* text);

  protected:
    aTEAvmLib			m_vmRef;
    aUILib			m_uiRef;
    aSettingFileRef		m_settings;
    
    acpStem**			m_pStems;

    acpHTMLSite*		m_pcViewSite;

    acpRobotOSFactory*		m_pcFactory;
    acpRobotVMManager*		m_pcVMM[aROBOT_APITEA_PROCESSCT];

    acpThread*			m_pcThread;    
    int				run();

 private:
    bool			m_bHB;
    aStreamRef			m_statusStream;
    aStreamRef			m_errorStream;

    acpRobot*			m_pcRobot;
    int				m_nDistanceUnitType;
    int				m_nAngleUnitType;
    int				m_nMassUnitType;
    
    acpMutex*			m_pcBehaviorMutex;
    acpList<acpRobotBehavior>	m_behaviors;
    acpRobotBehavior*		m_pCurrent;

    void			handlePacketOut (
    				  acpRobotPacketMessage* pMessage);

    static aErr 		sMultiHBProc (
    				  const aBool bHBOn,
				  void* ref);
    void			finishBehavior (
    				  const short status);

    void			dispatchPacketData (
  				  unsigned char address,
				  unsigned char length,
				  char* data);

    acpRobotPackage*		readPackage(
				  const char* pFileName);
    void			assignLinkID(
				  acpString& rName,
				  int* pnLinkID);
    void			collectTagsAndLinks(
				  acpRobotPackage* pPackage);
    void			createRobotObject(
				  acpPackageTag* pTag);
    void			initTagData();
    void			initValueFromString(
				  const char* pValStr,
				  const aPROPERTY_FLAGS flags,
				  acpValue* pVal);
    void			initConvFac();
    
    bool			m_bActive;
    bool			m_bSleeping;
    unsigned long		m_nHBInterval;
    unsigned long*		m_pnLastHB;
    unsigned long		m_nWakeUpTime;
    
    int				m_nNextBehaviorID;
    
    acpCallback*		m_pcHBCallback;

    acpMutex*			m_pcCallerMessageMutex;
    acpList<acpMessage>		m_callerMessages;

    int				m_numLinksUsed;
    acpList<acpString>		m_listLinksUsed;
    listPkgTags			m_tagList;

  friend class acpRobot;

  friend class acpRobotVMManager;
  friend class acpRobotPrimitive;
  friend class acpRobotProperty;
  friend class acpRobotUserProperty;
  friend class acpRobotUserObject;

  friend class acpRobotPacketMessage;
  friend class acpRobotVMLaunchMessage;
  friend class acpRobotVMKillMessage;
  friend class acpRobotCallbackMessage;

  friend class acpRobotHTMLMain;
  friend class acpRobotHTMLPrimitives;
  friend class acpRobotBehaviorList;
  friend class acpRobotScript;
  friend class acpRobotGlobal;
  friend class acpRobotSleep;
  friend class acpRobotMessage;

  friend class acpRobotHeartbeatCallbackProperty;
  friend class acpRobotHeartbeatStatusProperty;
  friend class acpRobotStatusStreamProperty;
  friend class acpRobotErrorStreamProperty;
};


/////////////////////////////////////////////////////////////////////

class acpRobotMessage :
  public acpMessage
{
  public:
                                acpRobotMessage(
                                  acpRobot* pcRobot,
                                  acpCallback* pcCallback) :
                                  m_pcRobot(pcRobot),
                                  m_pcCallback(pcCallback) {}

   void	                        process()
                                  { m_pcCallback->call(); }

  private:
    acpRobot*		        m_pcRobot;
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


#endif // _acpRobotInternal_H_
