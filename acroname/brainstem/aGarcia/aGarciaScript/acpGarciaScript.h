/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaScript.h                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the GarciaScript application         //
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

#ifndef _aGarciaScript_H_
#define _aGarciaScript_H_

#include "aUI.h"
#include "acpGarcia.h"
#include "aGarciaDefs.tea"


#define aCMD_DOSCRIPT			100
#define aCMD_DOHALT			101
#define aCMD_DOAPIVIEW			102
#define aCMD_DOEXIT			103
#define aLOGVIEWRECT			104
#define aSTATUSTEXT			105

#define aGS_PANE_COLOR_RED 		0
#define aGS_PANE_COLOR_GREEN		0
#define aGS_PANE_COLOR_BLUE		0
#define aGS_HBONLED_COLOR_RED 		0
#define aGS_HBONLED_COLOR_GREEN		255
#define aGS_HBONLED_COLOR_BLUE		50
#define aGS_HBOFFLED_COLOR_RED 		0
#define aGS_HBOFFLED_COLOR_GREEN	100
#define aGS_HBOFFLED_COLOR_BLUE		50



class acpGarciaScript {
  public:
  				acpGarciaScript();
    virtual			~acpGarciaScript();

    virtual void		die(const char* error) {}

    virtual void		welcome();

    virtual void		run() = 0;
    
    virtual void		setStatusText(const char* msg) = 0;

    virtual void		updateHeartbeat() = 0;

    virtual void		addLogLine(const char* msg)
    				  { aStream_WriteLine(m_ioRef, 
    				  		      m_logView, 
    				  		      msg, 
    				  		      NULL); }

    virtual void		cmdExit() {}

    virtual void		enableButton(const int nCmd,
    					     const bool bEnable) = 0;

    void			dispatchCommand(const int nCmd);

  protected:
    bool			m_bFinalized;
    bool			m_bHB;
    unsigned int		m_nTimerSetting;
    acpGarcia			m_garcia;
    int				m_batteryCheck;
    int				m_lastStatus;
    aIOLib			m_ioRef;
    aUILib			m_uiRef;
    aStreamRef			m_logView;

    void			TimeSlice();

  private:
    typedef enum eMode {
      kStarting,
      kInactive,
      kIdle,
      kRunning
    };
    // the garcia stuff
    void			cmdScript();
    void			cmdHalt();
    void			cmdAPIView();
    static aErr 		taskComplete(
  				  acpGarcia* pGarcia,
  				  acpObject* pBehavior);

    eMode			m_eMode;

    // the script and file dialog stuff
    static aBool		dumpFilter(const char* pFilename,
    					   const unsigned long nSize);
    static aErr			dialogIdle(const void* ref);

  friend class acpGarciaScriptHB;
};


class acpGarciaScriptHB :
  public acpCallback
{
  public:
	  		acpGarciaScriptHB(
  			  acpGarciaScript* pcScript) :
  			  m_pcScript(pcScript) {}

    aErr		call();
    
  private:
    acpGarciaScript* 	m_pcScript;
};

#endif // _aGarciaScript_H_
