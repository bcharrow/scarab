/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaTool.h                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the GarciaTool application           //
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


#ifndef _acpGarciaTool_H_
#define _acpGarciaTool_H_


#include "aIO.h"
#include "aUI.h"
#include "aStem.h"
#include "aGarciaGeom.h"
#include "aGarciaRF.h"


#define aGT_TITLE			"GarciaTool"
#define aGT_PANE_INSET			2
#define aGT_PANE_COLOR_RED 		100
#define aGT_PANE_COLOR_GREEN		200
#define aGT_PANE_COLOR_BLUE		130
#define aGT_HBONLED_COLOR_RED 		0
#define aGT_HBONLED_COLOR_GREEN		255
#define aGT_HBONLED_COLOR_BLUE		50
#define aGT_HBOFFLED_COLOR_RED 		0
#define aGT_HBOFFLED_COLOR_GREEN	100
#define aGT_HBOFFLED_COLOR_BLUE		50
#define aGT_PWRLED_COLOR_RED 		255
#define aGT_PWRLED_COLOR_GREEN		0
#define aGT_PWRLED_COLOR_BLUE		0
#define aGT_MAX_SETTING			32
#define aGT_IRTX_QUEUE_SIZE		10
#define aGT_SETTING_FILE		"garcia.config"
#define aGT_LED_RADIUS			11
#define aGT_NUM_CENTER_POINTS		9
#define aGT_DUMP_TXT			"Upgrade"
#define aGT_DUMP_CMD			10
#define aGT_RECOVER_TXT			"Recover"
#define aGT_RECOVER_CMD			11
#define aGT_USERLED_TXT			"User LED"
#define aGT_USERLED_CMD			12
#define aGT_STATS_TXT			"Stats"
#define aGT_STATS_CMD			13
// smaller text looks better on WinCE
#ifdef aWINCE
	#define aGT_RFLINK_TXT		"Init"
	#define aGT_RFREAD_TXT		"Read"
	#define aGT_RFWRITE_TXT		"Write"
	#define aGT_RFGPBAUD_TXT	"Set GP"
#else
	#define aGT_RFLINK_TXT		"Init link"
	#define aGT_RFREAD_TXT		"Read RF"
	#define aGT_RFWRITE_TXT		"Write RF"
	#define aGT_RFGPBAUD_TXT	"Set GP"
#endif
#define aGT_RFLINK_CMD			14
#define aGT_RFREAD_CMD			15
#define aGT_RFWRITE_CMD			16
#define aGT_RFGPBAUD_CMD		17

#define	aGT_RFBAUD_LABEL		"Baud"
#define	aGT_RFCHANNEL_LABEL		"Channel"

#define	aGT_RFBAUD_0			"2400"
#define	aGT_RFBAUD_1			"4800"
#define	aGT_RFBAUD_2			"9600"
#define	aGT_RFBAUD_3			"19200"
#define	aGT_RFBAUD_4			"38400"
#define	aGT_RFBAUD_5			"57600"
#define	aGT_RFBAUD_6			"115200"
#define	aGT_RFBOX_NONE			"(none)"

#define	aGT_MODAMP_LABEL		"Damp Const (ms)"
#define	aGT_MOPOS_LABEL			"+V"
#define	aGT_MONEG_LABEL			"-V"

#define	aGT_MOSTOP_TXT			"Stop"
#define	aGT_MOSTOPL_CMD			18
#define	aGT_MOSTOPR_CMD			19
#define	aGT_MOENCZERO_TXT		"Zero"
#define	aGT_MOENCZERO_CMD		20
#define	aGT_MODAMPENA_TXT		"Damping"
#define	aGT_MODAMPENA_CMD		21
#define	aGT_MODAMPSET_TXT		"Apply"
#define	aGT_MODAMPSET_CMD		22

#define aGT_IRDATA_LABEL		"TX Data (hex)"

#define	aGT_IRTXTEST_TXT		"Init test"
#define	aGT_IRTXTEST_CMD		23
#define	aGT_IRTXQUEUE_TXT		"Send queue"
#define	aGT_IRTXQUEUE_CMD		24
#define	aGT_IRQUEUEDATA_TXT		"Queue Data"
#define	aGT_IRQUEUEDATA_CMD		25

#define aGT_MOTOLINK_TXT		"Moto Link"
#define aGT_MOTOLINK_CMD		26

#define	aGT_DAMPVEL_MIN			2
#define	aGT_DAMPVEL_MAX			2000
#define	aGT_DAMPVEL_MIN_STR		"20"
#define	aGT_DAMPVEL_MAX_STR		"2000"



class acpGarciaTool {
  public:
  
    typedef enum {
      kView = 0,
      kConfig,
      kRF,
      kMoto,
      kIR
    } aGTPane;

    typedef enum {
      kCheckBoxLED = 0,
      kCheckBoxMoDamp
    } aGTCheckBox;

    typedef enum {
      kDropBoxRFBaud = 0,
      kDropBoxRFChannel
    } aGTDropBox;

    typedef enum {
      kSliderThrottleL = 0,
      kSliderThrottleR
    } aGTSlider;

    typedef enum {
      kParamIRTX = 0,
      kParamMoDamp
    } aGTParam;

  				acpGarciaTool();
    virtual			~acpGarciaTool();
    
    virtual void		buildUI() = 0;
    virtual bool		handleUIEvents() = 0;
    virtual void		switchPaneUI() = 0;
    virtual void		drawHB() = 0;
    virtual void		osSetCheckBox(
				  const int nWnd,
				  const int nState) = 0;
    virtual int			osGetCheckBox(
				  const int nWnd) const = 0;
    virtual void		osEnableWindowButton(
				  const int nWnd,
				  const int nCmd,
				  const bool bEnable) = 0;
    virtual void		osSetDropdownSel(
				  const int nWnd,
				  const int nSel) = 0;
    virtual int			osGetDropdownSel(
				  const int nWnd) const = 0;
    virtual void		osSetSlider(
				  const int nWnd,
				  const int nValue) = 0;
    virtual void		osGetText(
                                  const int nWnd,
                                  const int nBuffsize,
                                  char* pdata) const = 0;
    virtual void		osSetText(
				  const int nWnd,
				  char* pdata) = 0;
                                  
    virtual void		run();
    virtual void		setStatusText(const char* pText);
    virtual bool		handlePackets();
    virtual void		handleCommand(const int nCmd);
    virtual void		shutdown();
    virtual int			getUserLEDState();
    virtual void		tick();

    virtual void		addLogLineCfg(const char* msg)
    				  { aStream_WriteLine(m_ioRef, 
    				  		      m_logViewCfg, 
    				  		      msg, 
    				  		      NULL); }
    virtual void		addLogLineIR(const char* msg)
    				  { aStream_WriteLine(m_ioRef, 
    				  		      m_logViewIR, 
    				  		      msg, 
    				  		      NULL); }

  protected:
    void			changePane(
    				  const aGTPane ePane);
    virtual aErr		setMotionValue(
				  int nMotor,
				  int nValue);
    virtual void		queueValueForIRTX(int nValue);
    virtual aErr		setDualRampParams(
				  int nIndex,
				  int nValue);
    virtual void		setDampingConstant(const int n)
    				  { m_nDampVelConst = n; }

    aIOLib			m_ioRef;
    aUILib			m_uiRef;
    aStemLib			m_stemRef;
    aStreamRef			m_linkStream;
    aStreamRef			m_logViewCfg;
    aStreamRef			m_logViewIR;

    aSettingFileRef		m_settings;
    
    aRECT			m_rBattery;
    aGDRef			m_gdBattery;
    aRECT			m_rBot;
    aGDRef			m_gdBot;
    aRECT			m_rRFStatus;
    aGDRef			m_gdRFStatus;

    aRECT			m_rMotoEnc32L;
    aGDRef			m_gdMotoEnc32L;
    aRECT			m_rMotoEnc32R;
    aGDRef			m_gdMotoEnc32R;
    aRECT			m_rMotoAmpL;
    aGDRef			m_gdMotoAmpL;
    aRECT			m_rMotoAmpR;
    aGDRef			m_gdMotoAmpR;

    aRECT			m_rEdgeRight;
    aRECT			m_rEdgeLeft;
    aRECT			m_rButton;

    aBool			m_bDone;
    int				m_nWidth;
    int				m_nHeight;
    int				m_nBotWidth;
    int				m_nBotHeight;
    aBool			m_bHBOn;
    aGTPane			m_ePane;
    aGTPane			m_ePaneAssoc[10];
    int				m_nTabs;
    aBool			m_bShowRF;
  
  private:
    static aBool		dumpFilter(const char* pFilename,
    					   const unsigned long nSize);
    static aErr			showStatus(const char* pText,
    					   const void* ref);
    static aErr			dialogIdle(const void* ref);
    static aErr			rfIdle(const void* ref);
    static aErr			hbCallback(const aBool bHBOn,
    					   void* vpTool);
    void			delayWithUpdates(const int nTimeMS);
    void			doBrainDump();
    void			doRecover();
    void			doUserLED();
    void			doStats();
    void			doConfigMotoLink(
				  bool bRF);
    void			doRFRead();
    void			doRFWrite();
    aErr			doApplyGPBaud(
				  bool bReset,
				  bool bVerbose);
    aErr			doStopMotor(const int nMotor);
    void			doDampingControl();
    void			doApplyDampingConstant();
    void			doApplyIRTXData();
    void			doTestIRTX();
    void			doSendQueueIRTX();
    void			doEncZero();

    void			rangerEnable();
    void			setBattery(
    				  const float fVoltage);
    void			setMotoAmpGraph(
				  const int nMotor,
    				  const float fVoltage);
    void			setGDText(
				  aGDRef refGD,
				  aRECT* pRect,
				  const char* pText);

    void			redrawRobot();
    bool			m_bLinkOkay;
    bool			m_bMotoLink;
    int				m_nHBtimeout;
    unsigned long		m_ulNextTick;
    int				m_nTicksToBattery;
    int				m_nTicksToAmpRead;
    int				m_nTicksToEncRead;
    float			m_fBattery;
    aUIPixelType		m_xOffset;
    aUIPixelType		m_yOffset;
    aPT				m_pBotPoints[aNUMGARCIAOUTLINEPOINTS + 1];
    aPT				m_pCenterPoints[aGT_NUM_CENTER_POINTS];
    
    long			m_lMotoEnc32L;
    long			m_lMotoEnc32R;
    float			m_fRangers[6];
    int				m_nRangers[2];
    int				m_nButton;
    int				m_nUserLEDState;
    int				m_nDampVelState;
    int				m_nDampVelConst;
    int				m_nIRRX;
    aBool			m_bFrontRangers;
    aBool			m_bSideRangers;
    aBool			m_bRearRangers;
    aBool			m_bDownRangers;
    int				m_nIRTXqueue[aGT_IRTX_QUEUE_SIZE];
    int				m_nIRTXqueueCt;

    aGarciaRF*			m_pRF;
};

#endif // _aGarciaTool_H_
