/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotAgent.h                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the RobotAgent application           //
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

#ifndef _aRobotAgent_H_
#define _aRobotAgent_H_

#include "aUI.h"
#include "acpString.h"
#include "acpRobot.h"
#include "acpCallback.h"


#define aCMD_DOAPIVIEW			102
#define aCMD_DOEXIT			103
#define aLOGVIEWRECT			104
#define aSTATUSTEXT			105

#define aGA_PANE_COLOR_RED 		0
#define aGA_PANE_COLOR_GREEN		0
#define aGA_PANE_COLOR_BLUE		0
#define aGA_HBONLED_COLOR_RED 		0
#define aGA_HBONLED_COLOR_GREEN		255
#define aGA_HBONLED_COLOR_BLUE		50
#define aGA_HBOFFLED_COLOR_RED 		0
#define aGA_HBOFFLED_COLOR_GREEN	100
#define aGA_HBOFFLED_COLOR_BLUE		50

#define aROBOTAGENT_SETTINGSFILE	"aRobotAgent.config"
#define aROBOTAGENT_MAXSETTINGLEN       32
#define aROBOTAGENT_APINAMESETTING	"api_name"
#define aROBOTAGENT_DEFAULTPORT         8008


class acpAgentCBHook;

class acpRobotAgent {
  public:
  				acpRobotAgent();
    virtual			~acpRobotAgent();

    virtual void		die(const char* error) {}

    virtual void		welcome();

    virtual bool                init(
    				  const int argc,
    				  const char* argv[]);

    virtual void		run() = 0;
    
    virtual void		setStatusText(
				  const char* msg) = 0;

    virtual void		updateHeartbeat() = 0;

    virtual void		addLogLine(const char* msg)
//    				  { printf("log: %s\n", msg); }
#if 1
    				  { aStream_WriteLine(m_ioRef, 
    				  		      m_logView, 
    				  		      msg, 
    				  		      NULL); }
#endif

    virtual void		cmdExit() {}

    virtual void		enableButton(
				  const int nCmd,
				  const bool bEnable) = 0;

    void			dispatchCommand(
				  const int nCmd);

    void			TimeSlice();

  protected:
    aIOLib			m_ioRef;
    aUILib			m_uiRef;
    aSettingFileRef             m_settings;
    acpString                   m_name;
    bool			m_bFinalized;
    bool			m_bHB;
    unsigned int		m_nTimerSetting;
    acpRobot*			m_pRobot;
    int				m_batteryCheck;
    int				m_lastStatus;
    aStreamRef			m_logView;

    unsigned long		m_nInetAddr;    
    unsigned short		m_nPort;
    aStreamRef			m_socket;
    aStreamRef			m_buffer;

    unsigned long               m_linkUpTime;
    int                         m_linkUpInterval;
    
    void			addCallbackHook(
    				  acpAgentCBHook* pHook)
    				  { m_callbacks.addToTail(pHook); }

    virtual void		resetSocket();

  protected:
    // the robot stuff
    void			cmdAPIView();
    static aErr 		taskComplete(
  				  acpRobot* pRobot,
  				  acpObject* pBehavior);

    enum eRobotMode {
      kInit,
      kActive,
      kInactive
    };
    eRobotMode			m_eMode;
    bool			m_bConnected;
    int                         m_nUsers;

    acpList<acpAgentCBHook>	m_callbacks;
    int				m_nExecuteCBIndex;
    int				m_nCompletionCBIndex;
    int				m_nCompletionStatusIndex;

    void			handleRequest(
    				  char cFirstChar);
    void			respond(
    				  const char* pResponse);

    static aErr			propertyEnum(
    				  const char* pName,
    				  const int nIndex,
    				  aPROPERTY_FLAGS typeFlags,
    				  void* vpRef);

    static aErr			subObjectEnum(
    				  acpObject& object,
    				  void* vpRef);

    void			doAck();
    void			doSync();
    void			doProperties();
    void			doSubObjects();
    void			doName();
    void			doBehavior();
    void			doCallbacks();
    void			doRead();
    void			doTemp();
    void			doWrite();
    void                        doLinkUp();
    void                        doDescription();

    acpObject*			readObject();
    void			writeObject(
    				  const acpObject* pObject);
    short			readShort();
    void			writeShort(
    				  const short nVal);
    int				readInt();
    void			writeInt(
    				  const int nVal);
    void			writeFloat(
    				  const float fVal);
    unsigned char		readChar();
    void			writeChar(
    				  unsigned char pChar);
    void			readString(
    				  char* pString);
    void			writeString(
    				  const char* pString);
    float			readFloat();

    void			flush();

    // the script and file dialog stuff
    static aBool		dumpFilter(
    				  const char* pFilename,
    				  const unsigned long nSize);
    static aErr			dialogIdle(
    				  const void* ref);

  friend class acpRobotAgentHB;
  friend class acpRobotAgentExecute;
  friend class acpRobotAgentCompletion;
};


/////////////////////////////////////////////////////////////////////

class acpRobotAgentHB :
  public acpCallback
{
  public:
	  		acpRobotAgentHB(
  			  acpRobotAgent* pcAgent) :
  			  m_pcAgent(pcAgent) {}

    aErr		call();
 
  private:
    acpRobotAgent* 	m_pcAgent;
};

/////////////////////////////////////////////////////////////////////

class acpAgentCBHook :
  public acpCallback
{
  public:
	  		acpAgentCBHook(
  			  acpRobotAgent* pcAgent,
  			  acpObject* pBehavior,
  			  const int nCBID) :
  			  m_pcAgent(pcAgent),
  			  m_pBehavior(pBehavior),
  			  m_nCBID(nCBID) {}

    aErr		call() = 0;

    			// Override the release which normally
    			// delete's the callback.  We want
    			// to keep it around till the client
    			// asks for it.
    void		release() {}

    virtual aErr 	writeUpdateData(aStreamRef dest) 
    			  { return aErrNone; }

    int			getCBID()
    			  { return m_nCBID; }

  protected:
    acpRobotAgent* 	m_pcAgent;
    acpObject*		m_pBehavior;
    int			m_nCBID;
};


/////////////////////////////////////////////////////////////////////

class acpRobotAgentExecute :
  public acpAgentCBHook
{
  public:
	  		acpRobotAgentExecute(
  			  acpRobotAgent* pcAgent,
  			  acpObject* pBehavior,
  			  const int nCBID) :
  			  acpAgentCBHook(pcAgent, pBehavior, nCBID)
  			  {}
    aErr		call();

    virtual aErr 	writeUpdateData(aStreamRef dest); 
};


/////////////////////////////////////////////////////////////////////

class acpRobotAgentCompletion :
  public acpAgentCBHook
{
  public:
	  		acpRobotAgentCompletion(
  			  acpRobotAgent* pcAgent,
  			  acpObject* pBehavior,
  			  const int nCBID) :
  			  acpAgentCBHook(pcAgent, pBehavior, nCBID)
  			  {}
    aErr		call();

    virtual aErr 	writeUpdateData(aStreamRef dest); 

  private:
    int		 	m_nStatus;
};

#endif // _aRobotAgent_H_
