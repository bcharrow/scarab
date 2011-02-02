/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaTool.cpp                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GarciaTool application       //
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

#include "aStreamUtil.h"
#include "acpGarciaTool.h"
#include "aCmd.tea"
#include "aModule.tea"
#include "aBrainDump.h"
#include "aRecover.h"
#include "aGarciaRF.h"
#include "aGarciaDefs.tea"
#include "aUtil.h"
#include "aAnalog.h"
#include "aDigital.h"
#include "aPad.h"
#include "aCounter.h"
#include "aMotion.h"
#include "aModuleVal.h"
#include "aVersion.h"

#define aGT_WIDTH	240
#define aGT_HEIGHT	270

#define aGT_BAT_WIDTH	120
#define aGT_BAT_HEIGHT	12

#define aGT_EDGE_SENSOR_DIM		6
#define aGT_EDGE_SENSOR_XOFF		18
#define aGT_EDGE_SENSOR_YOFF		22

#define aGT_ROBOT_VIEW_WIDTH		220
#define aGT_ROBOT_VIEW_HEIGHT		180

#define aGT_GDFRAMECOLOR		0x888888

#define aGT_MSTICK_INC			80
#define aGT_MSHBTMR_TICKS		7
#define aGT_SCALING			8
#define aGT_CHUNKSIZE			16

#define	aGT_MO_GD_WIDTH			48


#define aSET_RF_STATUS_TEXT(x)	setGDText(m_gdRFStatus, &m_rRFStatus, x)
#define aSET_MO_ENC32L_TEXT(x)	setGDText(m_gdMotoEnc32L, &m_rMotoEnc32L, x)
#define aSET_MO_ENC32R_TEXT(x)	setGDText(m_gdMotoEnc32R, &m_rMotoEnc32R, x)



void charToLong(long* pn, const char* data);
void intToHexString(char* s, int n);
void hexStringToInt(int* pn, char* s);

void charToLong(long* pn, const char* data)
{
  long n = 0;
  n |= (((int)data[3]) & 0xFF);
  n |= (((int)data[2]) & 0xFF) << 8;
  n |= (((int)data[1]) & 0xFF) << 16;
  n |= (((int)data[0]) & 0xFF) << 24;
  *pn = n;
}

void intToHexString(char* s, int n)
{
  static char chex[17] = "0123456789ABCDEF";
  s[0] = chex[(n & 0xF000) >> 12];
  s[1] = chex[(n & 0x0F00) >> 8];
  s[2] = chex[(n & 0x00F0) >> 4];
  s[3] = chex[(n & 0x000F)];
  s[4] = '\0';
}

void hexStringToInt(int* pn, char* s)
{
  int i;
  int k = (int)aStringLen(s);
  int digit;
  int base = 1;
  int n = 0;
  for (i = 0; i < k; i++) {
    switch (s[k - i - 1]) {
      case '0': digit = 0; break;
      case '1': digit = 1; break;
      case '2': digit = 2; break;
      case '3': digit = 3; break;
      case '4': digit = 4; break;
      case '5': digit = 5; break;
      case '6': digit = 6; break;
      case '7': digit = 7; break;
      case '8': digit = 8; break;
      case '9': digit = 9; break;
      case 'a':
      case 'A': digit = 10; break;
      case 'b':
      case 'B': digit = 11; break;
      case 'c':
      case 'C': digit = 12; break;
      case 'd':
      case 'D': digit = 13; break;
      case 'e':
      case 'E': digit = 14; break;
      case 'f':
      case 'F': digit = 15; break;
      default:
	aAssert(0); /* shouldn't get here */
	digit = 0;
	break;
    }
    n += digit * base;
    base = base * 16;
  }
  *pn = n;
}


/////////////////////////////////////////////////////////////////////
// changePane method

void acpGarciaTool::changePane (
  const aGTPane ePane
)
{
  if (ePane == m_ePane)
    return;

  m_ePane = ePane;
  switchPaneUI();

  switch (m_ePane) {

    case kView:
      // switching to robot view will reset
      // enable circuits on robot
      rangerEnable();
      redrawRobot();
      break;

    case kConfig:
      // no link means user might need to try recover
      // (or use a link through the Moto port)
      // otherwise braindump and stats should be enabled
      osEnableWindowButton(kConfig, aGT_DUMP_CMD, m_bLinkOkay);
      osEnableWindowButton(kConfig, aGT_RECOVER_CMD, !m_bLinkOkay);
      osEnableWindowButton(kConfig, aGT_MOTOLINK_CMD, !m_bLinkOkay);
      osEnableWindowButton(kConfig, aGT_STATS_CMD, m_bLinkOkay);
      break;

    case kRF:
      break;

    case kMoto:
      break;

    case kIR:
      break;
  }

} // changePane method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool constructor

acpGarciaTool::acpGarciaTool() :
  m_linkStream(NULL),
  m_logViewCfg(NULL),
  m_logViewIR(NULL),
  m_gdBattery(NULL),
  m_bDone(false),
  m_nWidth(aGT_WIDTH),
  m_nHeight(aGT_HEIGHT),
  m_nBotWidth(aGT_ROBOT_VIEW_WIDTH),
  m_nBotHeight(aGT_ROBOT_VIEW_HEIGHT),  
  m_bHBOn(false),
  m_ePane(kView),
  m_nTabs(0),
  m_bShowRF(false),
  m_bLinkOkay(false),
  m_bMotoLink(false),
  m_nHBtimeout(0),
  m_nTicksToBattery(0),
  m_nTicksToAmpRead(0),
  m_nTicksToEncRead(0),
  m_fBattery(-1.0f),
  m_lMotoEnc32L(0),
  m_lMotoEnc32R(0),
  m_nUserLEDState(0),
  m_nDampVelState(0),
  m_nDampVelConst(aGT_DAMPVEL_MIN),
  m_nIRRX(0),
  m_bFrontRangers(true),
  m_bSideRangers(true),
  m_bRearRangers(true),
  m_bDownRangers(true),
  m_nIRTXqueueCt(0),
  m_pRF(NULL)
{
  aErr err = aErrNone;

  // set up some UI dimensions 
  m_rBot.x = 6;
  m_rBot.y = 10;
  m_rBot.width = aGT_ROBOT_VIEW_WIDTH;
  m_rBot.height = aGT_ROBOT_VIEW_HEIGHT;

  m_rBattery.x = 6;
  m_rBattery.y = 16 + aGT_ROBOT_VIEW_HEIGHT;
  m_rBattery.width = aGT_BAT_WIDTH;
  m_rBattery.height = aGT_BAT_HEIGHT;

  m_rRFStatus = m_rBattery;
  
  m_rMotoEnc32L.x = 24;
  m_rMotoEnc32L.y = 10;
  m_rMotoEnc32L.width = aGT_MO_GD_WIDTH;
  m_rMotoEnc32L.height = aGT_BAT_HEIGHT;
  
  m_rMotoEnc32R = m_rMotoEnc32L;
  m_rMotoAmpL = m_rMotoEnc32L;
  m_rMotoAmpR = m_rMotoEnc32L;
  m_rMotoEnc32R.x = 84;
  m_rMotoAmpR.x = 84;
  m_rMotoAmpL.y = aGT_ROBOT_VIEW_HEIGHT + 2;
  m_rMotoAmpR.y = m_rMotoAmpL.y;

  // find the center of the view
  m_xOffset = aGT_ROBOT_VIEW_WIDTH / 2;
  m_yOffset = aGT_ROBOT_VIEW_HEIGHT / 2;

  // set up edge sensors
  int edgecenter = aGT_EDGE_SENSOR_DIM / 2;
  m_rEdgeRight.x = (aUIPixelType)(m_xOffset + aGT_EDGE_SENSOR_XOFF - edgecenter);
  m_rEdgeRight.y = (aUIPixelType)(m_yOffset - aGT_EDGE_SENSOR_YOFF);
  m_rEdgeRight.width = aGT_EDGE_SENSOR_DIM;
  m_rEdgeRight.height = aGT_EDGE_SENSOR_DIM / 2;
  m_rEdgeLeft.x = (aUIPixelType)(m_xOffset - aGT_EDGE_SENSOR_XOFF - edgecenter);
  m_rEdgeLeft.y = (aUIPixelType)(m_yOffset - aGT_EDGE_SENSOR_YOFF);
  m_rEdgeLeft.width = aGT_EDGE_SENSOR_DIM;
  m_rEdgeLeft.height = aGT_EDGE_SENSOR_DIM / 2;


  // set up button state indicator
  m_rButton.x = (aUIPixelType)(m_xOffset);
  m_rButton.y = (aUIPixelType)(m_yOffset - 18);
  m_rButton.width = 3;
  m_rButton.height = 3;
  

  // set up the center indicator points
  aUIPixelType* d = (aUIPixelType*)m_pCenterPoints;
  *d++ = m_xOffset;		
  	*d++ = (aUIPixelType)(m_yOffset + 4);
  *d++ = (aUIPixelType)(m_xOffset - 1);	
  	*d++ = (aUIPixelType)(m_yOffset + 1);
  *d++ = (aUIPixelType)(m_xOffset - 4);	
  	*d++ = m_yOffset;
  *d++ = (aUIPixelType)(m_xOffset - 1);	
  	*d++ = (aUIPixelType)(m_yOffset - 1);
  *d++ = (aUIPixelType)m_xOffset;		
  	*d++ = (aUIPixelType)(m_yOffset - 4);
  *d++ = (aUIPixelType)(m_xOffset + 1);	
  	*d++ = (aUIPixelType)(m_yOffset - 1);
  *d++ = (aUIPixelType)(m_xOffset + 4);	
  	*d++ = m_yOffset;
  *d++ = (aUIPixelType)(m_xOffset + 1);	
  	*d++ = (aUIPixelType)(m_yOffset + 1);
  *d++ = m_xOffset;		
  	*d = (aUIPixelType)(m_yOffset + 4);

  // scale the geometry points to fit our drawing area
  d = (aUIPixelType*)m_pBotPoints;
  float* s = g_GarciaOutline;
  for (int i = 0; i < aNUMGARCIAOUTLINEPOINTS; i++) {
    *d++ = (aUIPixelType)(m_xOffset + *s++ * aGT_SCALING);
    *d++ = (aUIPixelType)(m_yOffset - *s++ * aGT_SCALING);
  }
  m_pBotPoints[aNUMGARCIAOUTLINEPOINTS] = m_pBotPoints[0];

  aIO_GetLibRef(&m_ioRef, &err);
  aAssert(err == aErrNone);
  aUI_GetLibRef(&m_uiRef, &err);
  aAssert(err == aErrNone);
  aStem_GetLibRef(&m_stemRef, &err);
  aAssert(err == aErrNone);

  // setting file
  aSettingFile_Create (
    m_ioRef, 
    aGT_MAX_SETTING, 
    aGT_SETTING_FILE,
    &m_settings, 
    &err);

  if (err == aErrNone) {
    char* pstr;
    aSettingFile_GetString(m_ioRef, 
    			m_settings, 
    			"rf-pane", 
    			&pstr,
    			"yes",
    			&err);
    if (!aStringCompare("yes", pstr)) {
      m_bShowRF = true;
    }
  }

  aGarciaRF_Create(&m_pRF,
		   m_ioRef,
		   m_uiRef,
		   m_stemRef,
		   aGARCIA_GP_ADDR,
		   rfIdle,
		   (void*)this);

  aAssert(err == aErrNone);

  aIO_GetMSTicks(m_ioRef, &m_ulNextTick, &err);
  m_ulNextTick += 1000;
  
} // acpGarciaTool constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaTool destructor

acpGarciaTool::~acpGarciaTool()
{
  aErr err = aErrNone;

  err = aGarciaRF_Destroy(&m_pRF);
  aAssert(err == aErrNone);
  if (m_logViewCfg) {
    aStream_Destroy(m_uiRef, m_logViewCfg, NULL);
    m_logViewCfg = NULL;
  }
  if (m_logViewIR) {
    aStream_Destroy(m_uiRef, m_logViewIR, NULL);
    m_logViewIR = NULL;
  }
  aAssert(err == aErrNone);
  if (m_gdBattery)
    aGD_Destroy(m_uiRef, m_gdBattery, &err);
  aAssert(err == aErrNone);
  if (m_gdBot)
    aGD_Destroy(m_uiRef, m_gdBot, &err);
  if (m_gdRFStatus)
    aGD_Destroy(m_uiRef, m_gdRFStatus, &err);
  if (m_gdMotoEnc32L)
    aGD_Destroy(m_uiRef, m_gdMotoEnc32L, &err);
  if (m_gdMotoEnc32R)
    aGD_Destroy(m_uiRef, m_gdMotoEnc32R, &err);
  aAssert(err == aErrNone);
  if (m_gdMotoAmpL)
    aGD_Destroy(m_uiRef, m_gdMotoAmpL, &err);
  if (m_gdMotoAmpR)
    aGD_Destroy(m_uiRef, m_gdMotoAmpR, &err);
  aAssert(err == aErrNone);
  aSettingFile_Destroy(m_ioRef, m_settings, &err);
  aAssert(err == aErrNone);
  aStem_ReleaseLibRef(m_stemRef, &err);
  aAssert(err == aErrNone);
  aUI_ReleaseLibRef(m_uiRef, &err);
  aAssert(err == aErrNone);
  aIO_ReleaseLibRef(m_ioRef, &err);
  aAssert(err == aErrNone);

} // acpGarciaTool destructor



/////////////////////////////////////////////////////////////////////
// handlePackets method

bool acpGarciaTool::handlePackets()
{
  bool bFound = false;
  
  if (m_linkStream) {
    aErr err = aErrNone;
    aPacketRef packet;
  
    aStem_GetPacket(m_stemRef, NULL, NULL, 0, &packet, &err);
    if (err == aErrNone) {
      bFound = true;
      aPacket_Destroy(m_stemRef, packet, &err);
    } else if ((err == aErrNotFound) || (err == aErrTimeout)) {
      err = aErrNone;
    } else {
      // other error, usually aErrIO
    }

//    aAssert(err == aErrNone);
  }

  return bFound;

} // acpGarciaTool handlePackets method



/////////////////////////////////////////////////////////////////////
// handleCommand method

void acpGarciaTool::handleCommand(const int nCmd)
{
  switch(nCmd) {

  case aGT_DUMP_CMD:
    doBrainDump();
    break;

  case aGT_RECOVER_CMD:
    doRecover();
    break;

  case aGT_USERLED_CMD:
    doUserLED();
    break;

  case aGT_STATS_CMD:
    doStats();
    break;

  case aGT_RFLINK_CMD:
    doConfigMotoLink(true);
    break;
  case aGT_MOTOLINK_CMD:
    doConfigMotoLink(false);
    break;

  case aGT_RFREAD_CMD:
    doRFRead();
    break;

  case aGT_RFWRITE_CMD:
    doRFWrite();
    break;

  case aGT_RFGPBAUD_CMD:
    doApplyGPBaud(aTrue, aTrue);
    break;

  case aGT_MOSTOPR_CMD:
    doStopMotor(aGARCIA_MOTO_MOTOR_RIGHT);
    break;

  case aGT_MOSTOPL_CMD:
    doStopMotor(aGARCIA_MOTO_MOTOR_LEFT);
    break;

  case aGT_MOENCZERO_CMD:
    doEncZero();
    break;

  case aGT_MODAMPENA_CMD:
    doDampingControl();
    break;

  case aGT_MODAMPSET_CMD:
    doApplyDampingConstant();
    break;

  case aGT_IRTXTEST_CMD:
    doTestIRTX();
    break;

  case aGT_IRTXQUEUE_CMD:
    doSendQueueIRTX();
    break;

  case aGT_IRQUEUEDATA_CMD:
    doApplyIRTXData();
    break;

  } // nCmd switch

} // acpGarciaTool handleCommand method



/////////////////////////////////////////////////////////////////////
// delayWithUpdates method

void acpGarciaTool::delayWithUpdates(const int nTimeMS)
{
  int i;
  unsigned long ulTick;
  unsigned long ulTickDone;
  aIO_GetMSTicks(m_ioRef, &ulTickDone, NULL);
  ulTickDone += (unsigned long)nTimeMS;

  for (i = 0; i < nTimeMS; i++) {
    if (!handleUIEvents() && !handlePackets())
      aIO_MSSleep(m_ioRef, 1, NULL);
    aIO_GetMSTicks(m_ioRef, &ulTick, NULL);
    if (ulTick > ulTickDone) break;
  }

} // acpGarciaTool delay method




/////////////////////////////////////////////////////////////////////
// run method

void acpGarciaTool::run()
{
  aErr err = aErrNone;

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

  // build the stream for communicating to garcia from the 
  // information in the settings file
  aStreamUtil_CreateSettingStream(m_ioRef,
    				  m_uiRef,
    				  m_stemRef,
    				  m_settings,
    				  &m_linkStream,
    				  &err);

  // set the heartbeat callback so we can handle heartbeats
  // in the UI and for timing.
  aStem_SetHBCallback(m_stemRef, hbCallback, this, &err);

  // force the ranger enables all on
  if (err == aErrNone)
    rangerEnable();

  // allow some time to establish heartbeat link
  // this makes sure we can query robot immediately
  int imax = 500;
  delayWithUpdates(imax);

  // determine initial state of user LED (could be on)
  m_nUserLEDState = getUserLEDState();
  osSetCheckBox(kCheckBoxLED, m_nUserLEDState);

  while (!m_bDone && (err == aErrNone)) {
    unsigned long now;

    aIO_GetMSTicks(m_ioRef, &now, &err);
    if (now > m_ulNextTick) {
      m_ulNextTick = now + aGT_MSTICK_INC - (now - m_ulNextTick);
      tick();
    }

    if (!handleUIEvents() && !handlePackets())
      aIO_MSSleep(m_ioRef, 1, NULL);
  }

} // acpGarciaTool run method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool shutdown method

void acpGarciaTool::shutdown()
{
  m_bDone = true;

} // acpGarciaTool shutdown method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool setBattery method

void acpGarciaTool::setBattery(const float fVoltage)
{
  aErr err = aErrNone;
  aRECT rIndicator = {
    0, 0, aGT_BAT_WIDTH - 20, aGT_BAT_HEIGHT
  };
  aRECT rBounds = m_rBattery;

  if (fVoltage == m_fBattery)
    return;

  float fCapacity = aGarciaGeom_VoltageToCapacity(fVoltage);

  if (err == aErrNone)
    aGD_StartDrawing(m_uiRef, m_gdBattery, &err);

  if (err == aErrNone)
    aGD_Erase(m_uiRef, m_gdBattery, &err);

  // draw a frame around the GD
  if (err == aErrNone)
    aGD_SetColor(m_uiRef, m_gdBattery, aGT_GDFRAMECOLOR, &err);
  if (err == aErrNone)
    aGD_Rect(m_uiRef, m_gdBattery, &rBounds, aFalse, &err);

  // inset for the text
  rBounds.x = rBounds.y = 1;
  rBounds.height -= 2;
  rBounds.width -= 4;

  if (err == aErrNone)
    aGD_SetColor(m_uiRef, m_gdBattery, 0x000000, &err);

  char voltage[10];
  aString_FormatFloat(fVoltage, voltage);
  aStringCat(voltage, "V");

  if (err == aErrNone)
    aGD_Text(m_uiRef, m_gdBattery, &rBounds, aUIALIGNRIGHT, 
    	     12, voltage, &err);
  
  if (err == aErrNone)
    aGD_Text(m_uiRef, m_gdBattery, &rBounds, aUIALIGNLEFT, 
    	     12, "battery", &err);

  if (err == aErrNone)
    aGD_SetColor(m_uiRef, m_gdBattery, 0x000000, &err);

  rIndicator.x += 34;
  rIndicator.width -= 50;
  rIndicator.y += 1;
  rIndicator.height -= 2;

  if (err == aErrNone)
    aGD_Rect(m_uiRef, m_gdBattery, &rIndicator, aFalse, &err);

  if (err == aErrNone)
    if (fCapacity < 0.1f)
      aGD_SetColor(m_uiRef, m_gdBattery, 0xFF0000, &err);
    else
      aGD_SetColor(m_uiRef, m_gdBattery, 0x00FF00, &err);

  rIndicator.x += 1;
  rIndicator.y += 1;
  rIndicator.height -= 2;
  rIndicator.width = (aUIPixelType)(rIndicator.width * fCapacity);
  if (err == aErrNone)
    aGD_Rect(m_uiRef, m_gdBattery, &rIndicator, aTrue, &err);

  if (err == aErrNone)
    aGD_EndDrawing(m_uiRef, m_gdBattery, &err);
  
  if (err == aErrNone)
    m_fBattery = fVoltage;

} // acpGarciaTool setBattery


#define COSF 0.407f
#define SINF 0.915f

/////////////////////////////////////////////////////////////////////
// acpGarciaTool redrawRobot method

void acpGarciaTool::redrawRobot()
{
  aErr err = aErrNone;

  if (err == aErrNone)
    aGD_StartDrawing(m_uiRef, m_gdBot, &err);

  if (err == aErrNone)
    aGD_Erase(m_uiRef, m_gdBot, &err);

  // draw a frame around the GD
  if (err == aErrNone)
    aGD_SetColor(m_uiRef, m_gdBot, aGT_GDFRAMECOLOR, &err);
  if (err == aErrNone) {
    aRECT r = m_rBot;
    r.x = r.y = 0;
    aGD_Rect(m_uiRef, m_gdBot, &r, aFalse, &err);
  }

  if (err == aErrNone)
    aGD_SetColor(m_uiRef, m_gdBot, 0x0000FF, &err);

  if (err == aErrNone)
    aGD_Line(m_uiRef, m_gdBot, m_pBotPoints,
             aNUMGARCIAOUTLINEPOINTS + 1, &err);

  if (err == aErrNone)
    aGD_SetColor(m_uiRef, m_gdBot, 0xFF0000, &err);

  if (err == aErrNone)
    aGD_Line(m_uiRef, m_gdBot, m_pCenterPoints, 
    	     aGT_NUM_CENTER_POINTS, &err);


  aPT pt[2];

  if (m_bFrontRangers) {
    // the front rangers
    static float xDim = (0.5f * SINF * aGT_SCALING);
    static float yDim = (0.5f * COSF * aGT_SCALING);
    float xDist, yDist;
    if (m_fRangers[0]) {
      xDist = (m_fRangers[0] * aGT_SCALING * COSF);
      yDist = (m_fRangers[0] * aGT_SCALING * SINF);
      pt[0].x = (aUIPixelType)(m_xOffset - (xDist - xDim));
      pt[0].y = (aUIPixelType)(m_yOffset - (yDist + yDim));
      pt[1].x = (aUIPixelType)(m_xOffset - (xDist + xDim));
      pt[1].y = (aUIPixelType)(m_yOffset - (yDist - yDim));
      if (err == aErrNone)
        aGD_Line(m_uiRef, m_gdBot, pt, 2, &err);
    }
    if (m_fRangers[1]) {
      xDist = (m_fRangers[1] * aGT_SCALING * COSF);
      yDist = (m_fRangers[1] * aGT_SCALING * SINF);
      pt[0].x = (aUIPixelType)(m_xOffset + (xDist + xDim));
      pt[0].y = (aUIPixelType)(m_yOffset - (yDist - yDim));
      pt[1].x = (aUIPixelType)(m_xOffset + (xDist - xDim));
      pt[1].y = (aUIPixelType)(m_yOffset - (yDist + yDim));
      if (err == aErrNone)
        aGD_Line(m_uiRef, m_gdBot, pt, 2, &err);
    }
  }

  if (m_bSideRangers) { 
    pt[0].y = (aUIPixelType)(m_yOffset - 3.5f * aGT_SCALING);
    pt[1].y = (aUIPixelType)(m_yOffset - 2.5f * aGT_SCALING);
    if (m_fRangers[2]) {
      pt[0].x = (aUIPixelType)(m_xOffset - m_fRangers[2] * aGT_SCALING);
      pt[1].x = pt[0].x;
      if (err == aErrNone)
        aGD_Line(m_uiRef, m_gdBot, pt, 2, &err);
    }
    if (m_fRangers[3]) {
      pt[0].x = (aUIPixelType)(m_xOffset + m_fRangers[3] * aGT_SCALING);
      pt[1].x = pt[0].x;
      if (err == aErrNone)
        aGD_Line(m_uiRef, m_gdBot, pt, 2, &err);
    }
  }

  if (m_bRearRangers) {
    if (m_fRangers[4]) {
      pt[0].x = (aUIPixelType)(m_xOffset - 2.375f * aGT_SCALING);
      pt[0].y = (aUIPixelType)(m_yOffset + m_fRangers[4] * aGT_SCALING);
      pt[1].x = (aUIPixelType)(m_xOffset - 3.375f * aGT_SCALING);
      pt[1].y = pt[0].y;
      if (err == aErrNone)
        aGD_Line(m_uiRef, m_gdBot, pt, 2, &err);
    }
    if (m_fRangers[5]) {
      pt[0].x = (aUIPixelType)(m_xOffset + 2.375f * aGT_SCALING);
      pt[0].y = (aUIPixelType)(m_yOffset + m_fRangers[5] * aGT_SCALING);
      pt[1].x = (aUIPixelType)(m_xOffset + 3.375f * aGT_SCALING);
      pt[1].y = pt[0].y;
      if (err == aErrNone)
        aGD_Line(m_uiRef, m_gdBot, pt, 2, &err);
    }
  }

  // indicator for user button state
  if (err == aErrNone) {
    if (m_nButton)
      aGD_SetColor(m_uiRef, m_gdBot, 0x000000, &err);
    else
      aGD_SetColor(m_uiRef, m_gdBot, 0xC0C0C0, &err);
    if (err == aErrNone)
      aGD_Rect(m_uiRef, m_gdBot, &m_rButton, aFalse, &err);
  }


  // indicators for down sensor state
  if (m_bDownRangers) {
    if (err == aErrNone) {
      if (m_nRangers[0])
        aGD_SetColor(m_uiRef, m_gdBot, 0xFF0000, &err);
      else
        aGD_SetColor(m_uiRef, m_gdBot, 0x00FF00, &err);
      if (err == aErrNone) {
        aGD_Rect(m_uiRef, m_gdBot, &m_rEdgeRight, aTrue, &err);
      }
    }
    if (err == aErrNone) {
      if (m_nRangers[1])
        aGD_SetColor(m_uiRef, m_gdBot, 0xFF0000, &err);
      else
        aGD_SetColor(m_uiRef, m_gdBot, 0x00FF00, &err);
      if (err == aErrNone) {
        aGD_Rect(m_uiRef, m_gdBot, &m_rEdgeLeft, aTrue, &err);
      }
    }
  }

  if (err == aErrNone)
    aGD_EndDrawing(m_uiRef, m_gdBot, &err);

} // acpGarciaTool redrawRobot



/////////////////////////////////////////////////////////////////////
// acpGarciaTool tick method

void acpGarciaTool::tick()
{
  aErr err = aErrNone;
  char buff[32];

  if (!m_linkStream)
    return;

//  if (aGT_HB_ACTIVE)
//    m_nHBtimeout++;

  // manage link status flag
  if (m_nHBtimeout == aGT_MSHBTMR_TICKS) {
    m_bLinkOkay = false;
  } else if (m_nHBtimeout < aGT_MSHBTMR_TICKS) {
    m_nHBtimeout++;
  }
  
  switch (m_ePane) {
  
  case kView:
    // every sixty ticks we check the battery voltage
    if (m_bLinkOkay && (m_nTicksToBattery-- <= 0)) {
      int nVal;
      float fVoltage = 0.0f;
      err = aAnalog_ReadInt(m_stemRef, 
                        aGARCIA_GP_ADDR,
                        aGARCIA_GP_ABATTERY,
                        &nVal);
      if (err == aErrNone) {
        fVoltage = (float)nVal / aGARCIA_BATT_VOLTAGE_CONVFAC;
        m_nTicksToBattery = 60;
      }
      setBattery(fVoltage);
    }

    int val;

    if ((err == aErrNone) && m_bFrontRangers && m_bLinkOkay) {
      // front left
      err = aAnalog_ReadInt(m_stemRef,
  		  aGARCIA_MOTO_ADDR, 
  		  aGARCIA_MOTO_ARANGE_FRONT_LEFT,
  		  &val);
      m_fRangers[0] = aGarciaGeom_ScaleRangerData((unsigned short)val, kGarciaGeomFrontRanger, 
  					      aGARCIA_DISTANCE_INCHES);

      // front right
      err = aAnalog_ReadInt(m_stemRef,
  		  aGARCIA_MOTO_ADDR, 
  		  aGARCIA_MOTO_ARANGE_FRONT_RIGHT,
  		  &val);
      m_fRangers[1] = aGarciaGeom_ScaleRangerData((unsigned short)val, kGarciaGeomFrontRanger, 
  					      aGARCIA_DISTANCE_INCHES);
    }

    if ((err == aErrNone) && m_bSideRangers && m_bLinkOkay) {
      // side left
      err = aAnalog_ReadInt(m_stemRef,
  		  aGARCIA_MOTO_ADDR, 
  		  aGARCIA_MOTO_ARANGE_SIDE_LEFT,
  		  &val);
      m_fRangers[2] = aGarciaGeom_ScaleRangerData((unsigned short)val, kGarciaGeomSideRanger, 
  					      aGARCIA_DISTANCE_INCHES);
      // side right
      err = aAnalog_ReadInt(m_stemRef,
  		  aGARCIA_MOTO_ADDR, 
  		  aGARCIA_MOTO_ARANGE_SIDE_RIGHT,
  		  &val);
      m_fRangers[3] = aGarciaGeom_ScaleRangerData((unsigned short)val, kGarciaGeomSideRanger, 
  					      aGARCIA_DISTANCE_INCHES);
    }


    if ((err == aErrNone) && m_bRearRangers && m_bLinkOkay) {
      // back left
      err = aAnalog_ReadInt(m_stemRef,
  		  aGARCIA_GP_ADDR, 
  		  aGARCIA_GP_ARANGE_REAR_LEFT,
  		  &val);
      m_fRangers[4] = aGarciaGeom_ScaleRangerData((unsigned short)val, kGarciaGeomRearRanger, 
  					      aGARCIA_DISTANCE_INCHES);
      // back right
      err = aAnalog_ReadInt(m_stemRef,
  		  aGARCIA_GP_ADDR, 
  		  aGARCIA_GP_ARANGE_REAR_RIGHT,
  		  &val);
      m_fRangers[5] = aGarciaGeom_ScaleRangerData((unsigned short)val, kGarciaGeomRearRanger, 
  					      aGARCIA_DISTANCE_INCHES);
    }


    if ((err == aErrNone) && m_bDownRangers && m_bLinkOkay) {
      // down right
      err = aDigital_ReadInt(m_stemRef,
  		  aGARCIA_MOTO_ADDR, 
  		  aGARCIA_MOTO_DEDGE_RIGHT,
  		  &val);
      m_nRangers[0] = val;
      // down left
      err = aDigital_ReadInt(m_stemRef,
  		  aGARCIA_MOTO_ADDR, 
  		  aGARCIA_MOTO_DEDGE_LEFT,
  		  &val);
      m_nRangers[1] = val;
    }

    if ((err == aErrNone) && m_bLinkOkay) {
      // button
      err = aDigital_ReadInt(m_stemRef,
  		  aGARCIA_GP_ADDR, 
  		  aGARCIA_GP_DBUTTON,
  		  &val);
      m_nButton = val;
    }

    redrawRobot();
    break;

  case kConfig:
    break;

  case kRF:
    break;

  case kMoto:

    // every 2 ticks we check the moto amp readings
    if (m_bLinkOkay && (m_nTicksToAmpRead-- <= 0)) {

      int nCurrentL;
      int nCurrentR;

      err = aAnalog_ReadInt(m_stemRef, 
                        aGARCIA_GP_ADDR,
                        aGARCIA_GP_ACURRENT_LMOTOR,
                        &nCurrentL);
      err = aAnalog_ReadInt(m_stemRef, 
                        aGARCIA_GP_ADDR,
                        aGARCIA_GP_ACURRENT_RMOTOR,
                        &nCurrentR);
      if (err == aErrNone) {
        m_nTicksToAmpRead = 1;
        setMotoAmpGraph(aGARCIA_MOTO_MOTOR_LEFT, ((float)nCurrentL)/1024.0f);
        setMotoAmpGraph(aGARCIA_MOTO_MOTOR_RIGHT, ((float)nCurrentR)/1024.0f);
      }
    }

    // every 10 ticks we check the encoders
    if (m_bLinkOkay && (m_nTicksToEncRead-- <= 0)) {

      err = aMotion_GetEnc32(m_stemRef, 
                        (unsigned char)aGARCIA_MOTO_ADDR,
                        (unsigned char)aGARCIA_MOTO_MOTOR_LEFT,
                        buff);
      charToLong(&m_lMotoEnc32L, buff);
      err = aMotion_GetEnc32(m_stemRef, 
                        (unsigned char)aGARCIA_MOTO_ADDR,
                        (unsigned char)aGARCIA_MOTO_MOTOR_RIGHT,
                        buff);
      charToLong(&m_lMotoEnc32R, buff);

      if (err == aErrNone) {
        m_nTicksToEncRead = 10;
        aStringFromInt(buff, (int)m_lMotoEnc32L);
        aSET_MO_ENC32L_TEXT(buff);
        aStringFromInt(buff, (int)m_lMotoEnc32R);
        aSET_MO_ENC32R_TEXT(buff);
      }
    }
    break;

  case kIR:
    if (m_bLinkOkay) {
      short nIR;
      err = aCounter_ReadShort(m_stemRef, 
                        (unsigned char)aGARCIA_MOTO_ADDR,
                        (unsigned char)aGARCIA_MOTO_CTR_IRRX,
                        &nIR);
      // check for non-zero input
      // reset counter back to zero after reading input
      if ((nIR != 0) && (err == aErrNone)) {
        m_nIRRX = nIR;
        err = aCounter_WriteShort(m_stemRef, 
                        (unsigned char)aGARCIA_MOTO_ADDR,
                        (unsigned char)aGARCIA_MOTO_CTR_IRRX,
                        0);
        intToHexString(buff, nIR);
        addLogLineIR(buff);
      }
    }
    break;

  } // m_ePane switch

} // acpGarciaTool tick



/////////////////////////////////////////////////////////////////////
// acpGarciaTool hbCallback static method

aErr acpGarciaTool::hbCallback (
  const aBool bHBOn,
  void* vpTool
)
{
  acpGarciaTool* pTool = (acpGarciaTool*)vpTool;

  aAssert(pTool);
  
  pTool->m_nHBtimeout = 0;
  pTool->m_bLinkOkay = true;
  pTool->m_bHBOn = (bHBOn == aTrue);

  pTool->drawHB();
  
  pTool->handleUIEvents();

  return aErrNone;

} // acpGarciaTool hbCallback static method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool dumpFilter method

aBool acpGarciaTool::dumpFilter(
  const char* pFilename,
  const unsigned long nSize
)  
{
  char extension[aFILE_NAMEMAXCHARS];
  
  aUtil_GetFileExtension(extension, pFilename, NULL);
  if (!aStringCompare(extension, ".dump"))
    return aTrue;
  
  return aFalse;

} // acpGarciaTool dumpFilter method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool showStatus method

aErr acpGarciaTool::showStatus (
  const char* pText,
  const void* ref
)  
{
  acpGarciaTool* pTool = (acpGarciaTool*)ref;

  aAssert(pTool);

  pTool->handleUIEvents();
  pTool->handlePackets();

  if (pText)
    pTool->setStatusText(pText);

  return aErrNone;

} // acpGarciaTool showStatus method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool setStatusText method

void acpGarciaTool::setStatusText (
  const char* pText
)
{
  addLogLineCfg(pText);
  
} // acpGarciaTool::setStatusText method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool::dialogIdle static method

aErr acpGarciaTool::dialogIdle(const void* ref)
{
  acpGarciaTool* pTool = (acpGarciaTool*)ref;

  aAssert(pTool);
  
  pTool->handlePackets();
  
  return aErrNone;
  
} // acpGarciaTool::dialogIdle static method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool::rfIdle static method

aErr acpGarciaTool::rfIdle(const void* ref)
{
  acpGarciaTool* pTool = (acpGarciaTool*)ref;
  aAssert(pTool);
  pTool->handleUIEvents();
  pTool->handlePackets();
  return aErrNone;
  
} // acpGarciaTool::rfIdle static method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doBrainDump method

void acpGarciaTool::doBrainDump()
{
  aErr err;
  char filename[aFILE_NAMEMAXCHARS];
  aStreamRef fileStream = NULL;
  aBDRef bd = NULL;

  // disable buttons
  // (recover is already disabled if stats has been called)
  osEnableWindowButton(kConfig, aGT_DUMP_CMD, aFalse);
  osEnableWindowButton(kConfig, aGT_STATS_CMD, aFalse);
  osEnableWindowButton(kConfig, aGT_MOTOLINK_CMD, aFalse);

  addLogLineCfg("------------------------");
  addLogLineCfg("Upgrade");

  aDialog_PickFile(m_uiRef,
    		   "Unlock EEPROM.  Choose File.",
    		   filename,
    		   aFileAreaObject,
    		   dumpFilter,
    		   dialogIdle,
    		   this,
    		   &err);

  // re-establish heartbeat (Mac seems to lose it momentarily after file dialog)
  delayWithUpdates(100);


  if (err == aErrNone)
    aStream_CreateFileInput(m_ioRef, 
    		   	    filename, 
    		   	    aFileAreaObject,
    		   	    &fileStream,
    			    &err);

  if (err == aErrNone)
    err = aBD_Create(m_ioRef, m_stemRef, showStatus, (void*)this, &bd);

  if (err == aErrNone)
    err = aBD_Read(bd, fileStream);

  if (err == aErrNone)
    err = aBD_CheckLock(bd, aGARCIA_GP_ADDR, aGARCIA_LOCK_CHK_ADDR);

  if (err == aErrNone)
    err = aBD_Dump(bd);

  if (err != aErrNone) {
    addLogLineCfg("Upgrade FAILED");
  } else {
    addLogLineCfg("Upgrade SUCCESSFUL!");
  }

  if (bd)
    aBD_Destroy(bd);

  // clean up the file either way if it was opened
  if (fileStream != NULL) {
    aStream_Destroy(aStreamLibRef(fileStream), fileStream, NULL);
    fileStream = NULL;
  }

  // re-enable buttons
  // (recover is already disabled if stats has been called)
  // (moto link will stay disabled once link is made)
  osEnableWindowButton(kConfig, aGT_DUMP_CMD, aTrue);
  osEnableWindowButton(kConfig, aGT_STATS_CMD, aTrue);
  
  // restore normal network settings
  // since braindump saves the Moto link settings
  // restore temporary Moto link
  if (m_bMotoLink) {

    if (err == aErrNone)
      err = aModuleVal_Set(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_ROUTER,
			 aGARCIA_GP_ADDR);
    if (err == aErrNone)
      err = aModuleVal_Set(m_stemRef,
			 aGARCIA_MOTO_ADDR,
			 aMODULE_VAL_ROUTER,
			 aGARCIA_GP_ADDR);
    if (err == aErrNone)
      err = aModuleVal_Save(m_stemRef, aGARCIA_GP_ADDR);
    if (err == aErrNone)
      err = aModuleVal_Save(m_stemRef, aGARCIA_MOTO_ADDR);

    if (err == aErrNone)
      err = aModuleVal_Set(m_stemRef,
			 aGARCIA_MOTO_ADDR,
			 aMODULE_VAL_ROUTER,
			 aGARCIA_MOTO_ADDR);
    if (err == aErrNone)
      err = aModuleVal_Set(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_ROUTER,
			 aGARCIA_MOTO_ADDR);

    // re-establish heartbeat
    delayWithUpdates(100);
  }

} // acpGarciaTool doBrainDump method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doRecover method

void acpGarciaTool::doRecover()
{

  aErr err = aErrNone;
  aRecRef recRef = NULL;
  
  char module1;
  char module2;
  char type1;

  // disable recover button
  // (braindump and stats are already disabled if here)
  osEnableWindowButton(kConfig, aGT_RECOVER_CMD, aFalse);

  addLogLineCfg("------------------------");
  addLogLineCfg("Recovery");
    
  if (err == aErrNone)
    err = aRecover_Create(m_ioRef, 
    			  m_stemRef, 
    			  m_linkStream, 
    			  m_settings, 
    			  showStatus,
    			  (void*)this,
    			  &recRef);

  if (err == aErrNone)
    err = aRecover_Identify(recRef,
    			    &m_bDone,     
    			    NULL, &module1, &type1);

  if (err == aErrNone)
    err = aRecover_SeekNetworkStem(recRef, 
    				   &m_bDone, 
    				   module1, 
    				   0, 
    				   &module2);
    
  /* re-establish default garcia network */
  if (err == aErrNone && !m_bDone) {
    if (type1 == aMODULE_TYPE_MOTO) {

      aAssert((aGARCIA_GP_ADDR + 14) <= 254);

      /* do a little shuffle if there is an address conflict */
      unsigned char tempaddr;
      if (module1 == aGARCIA_GP_ADDR) {
        if (module2 == (aGARCIA_GP_ADDR + 12)) {
          tempaddr = (aGARCIA_GP_ADDR + 14);        
        } else {
          tempaddr = (aGARCIA_GP_ADDR + 12);
        }
        if (err == aErrNone) err = aModuleVal_Set(m_stemRef, (unsigned char)module1, aMODULE_VAL_ROUTER, (char)tempaddr);
        if (err == aErrNone) err = aModuleVal_Set(m_stemRef, (unsigned char)module1, aMODULE_VAL_ADDRESS, (char)tempaddr);
        if (err == aErrNone) err = aModuleVal_Set(m_stemRef, (unsigned char)module2, aMODULE_VAL_ROUTER, (char)tempaddr);
          aIO_MSSleep(m_ioRef, 500, NULL);
      } else {
        tempaddr = (unsigned char)module1;
      }

      /* reset networked GP */
      if (err == aErrNone)
        err = aRecover_ResetParams(recRef,
				 &m_bDone,
				 (unsigned char)module2,
				 aGARCIA_GP_ADDR,
				 aGARCIA_GP_ADDR,
				 aMODULE_BAUDRATE_38400,
				 0,
				 aTrue,
				 aTrue);
      /* reset routing MOTO */
      if (err == aErrNone)
        err = aRecover_ResetParams(recRef,
				 &m_bDone,
				 (unsigned char)tempaddr,
				 aGARCIA_MOTO_ADDR,
				 aGARCIA_GP_ADDR,
				 aMODULE_BAUDRATE_9600,
				 0,
				 aTrue,
				 aTrue);
    } else {

      aAssert((aGARCIA_MOTO_ADDR + 14) <= 254);

      /* do a little shuffle if there is an address conflict */
      unsigned char tempaddr;
      if (module1 == aGARCIA_MOTO_ADDR) {
        if (module2 == (aGARCIA_MOTO_ADDR + 12)) {
          tempaddr = (aGARCIA_MOTO_ADDR + 14);        
        } else {
          tempaddr = (aGARCIA_MOTO_ADDR + 12);
        }
        if (err == aErrNone) err = aModuleVal_Set(m_stemRef, (unsigned char)module1, aMODULE_VAL_ROUTER, (char)tempaddr);
        if (err == aErrNone) err = aModuleVal_Set(m_stemRef, (unsigned char)module1, aMODULE_VAL_ADDRESS, (char)tempaddr);
        if (err == aErrNone) err = aModuleVal_Set(m_stemRef, (unsigned char)module2, aMODULE_VAL_ROUTER, (char)tempaddr);
          aIO_MSSleep(m_ioRef, 500, NULL);
      } else {
        tempaddr = (unsigned char)module1;
      }

      /* reset networked MOTO */
      if (err == aErrNone)
        err = aRecover_ResetParams(recRef,
				 &m_bDone,
				 (unsigned char)module2,
				 aGARCIA_MOTO_ADDR,
				 aGARCIA_GP_ADDR,
				 aMODULE_BAUDRATE_9600,
				 0,
				 aTrue,
				 aTrue);
      /* reset routing GP */
      if (err == aErrNone)
        err = aRecover_ResetParams(recRef,
				 &m_bDone,
				 (unsigned char)tempaddr,
				 aGARCIA_GP_ADDR,
				 aGARCIA_GP_ADDR,
				 aMODULE_BAUDRATE_38400,
				 0,
				 aTrue,
				 aTrue);
    }
  }


  // check for success from either Moto or GP serial connection
  if (!m_bDone) {
    if ((err == aErrNone) &&
        ((module1 == aGARCIA_MOTO_ADDR && module2 == aGARCIA_GP_ADDR) ||
        (module1 == aGARCIA_GP_ADDR && module2 == aGARCIA_MOTO_ADDR))) {

      addLogLineCfg("Recovery SUCCESSFUL!");
      addLogLineCfg("Turn off robot.");
      addLogLineCfg("Restart GarciaTool.");
      addLogLineCfg("Perform BrainDump.");

    } else {

      addLogLineCfg("Recovery FAILED.");

      // kill whatever old stream was left by recovery procedure
      aStem_SetStream(m_stemRef, NULL, kStemModuleStream, &err);

      // rebuild stream for communicating with garcia from
      // the information in the settings file
      aStreamUtil_CreateSettingStream(m_ioRef,
				      m_uiRef,
				      m_stemRef,
				      m_settings,
				      &m_linkStream,
				      &err);
      // re-enable recover button
      osEnableWindowButton(kConfig, aGT_RECOVER_CMD, aTrue);
    }
  }

  if (recRef)
    aRecover_Destroy(recRef);

} // acpGarciaTool doRecover method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool getUserLEDState method

int acpGarciaTool::getUserLEDState()
{
  char c;
  aErr err;

  err = aPad_ReadChar(m_stemRef,
		      aGARCIA_MOTO_ADDR,
		      aGARCIA_MOTO_PADB_MIRRORIO,
		      &c);

  if (err == aErrNone)
    m_nUserLEDState = c;

  return m_nUserLEDState;

} // acpGarciaTool getUserLEDState method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doUserLED method

void acpGarciaTool::doUserLED()
{
  aErr err;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];

  m_nUserLEDState = osGetCheckBox(kCheckBoxLED);

  data[0] = cmdRAW_INPUT;
  data[1] = aGARCIA_MOTO_RFLX_LED;
  data[2] = (char)m_nUserLEDState;

  aPacket_Create(m_stemRef, aGARCIA_MOTO_ADDR, 3, data, &packet, &err);
  aStem_SendPacket(m_stemRef, packet, &err);

}  // acpGarciaTool doUserLED method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doStats method

void acpGarciaTool::doStats()
{
  int nGPsn = 0;
  int nMOTOsn = 0;
  int nGPbld = 0;
  int nMOTObld = 0;
  aErr err;
  char buff[80];
  char cuff[20];
  char data[aSTEMMAXPACKETBYTES];

  // disable buttons
  // (recover is already disabled if stats has been called)
  osEnableWindowButton(kConfig, aGT_DUMP_CMD, aFalse);
  osEnableWindowButton(kConfig, aGT_STATS_CMD, aFalse);

  addLogLineCfg("------------------------");
  addLogLineCfg("Garcia Stats");

  aStringCopy(buff, "Software Build ");
  aStringFromInt(cuff, aGARCIA_BUILD_NUM);
  aStringCat(buff, cuff);
  addLogLineCfg(buff);
  
  err = aModuleVal_Get(m_stemRef,
		       aGARCIA_GP_ADDR,
		       aMODULE_VAL_SNLOW,
		       &data[0]);

  if (err == aErrNone)
    err = aModuleVal_Get(m_stemRef,
			 aGARCIA_MOTO_ADDR,
			 aMODULE_VAL_SNLOW,
			 &data[2]);

  if (err == aErrNone) {
    nGPsn = aUtil_RetrieveShort(&data[0]);
    nMOTOsn = aUtil_RetrieveShort(&data[2]);
    sprintf(buff, "Garcia ID:  %04X%04X", nGPsn, nMOTOsn);
    addLogLineCfg(buff);
  }

  if (err == aErrNone)
    err = aModuleVal_Get(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_BUILD,
			 &data[0]);

  if (err == aErrNone) {
    nGPbld = aUtil_RetrieveShort(&data[0]);
    err = aModuleVal_Get(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_VERSION,
			 &data[2]);
    sprintf(buff, "  GP %i.%i %i", data[2], data[3], nGPbld);
    addLogLineCfg(buff);
  }

  if (err == aErrNone)
    err = aModuleVal_Get(m_stemRef,
			 aGARCIA_MOTO_ADDR,
			 aMODULE_VAL_BUILD,
			 &data[0]);

  if (err == aErrNone) {
    nMOTObld = aUtil_RetrieveShort(&data[0]);
    err = aModuleVal_Get(m_stemRef,
			 aGARCIA_MOTO_ADDR,
			 aMODULE_VAL_VERSION,
 			 &data[2]);
    aStringCopy(buff, "Moto ");
    aStringFromInt(cuff, data[2]);
    aStringCat(buff, cuff);
    aStringCat(buff, ".");
    aStringFromInt(cuff, data[3]);
    aStringCat(buff, cuff);
    aStringCat(buff, " ");
    aStringFromInt(cuff, nMOTObld);
    aStringCat(buff, cuff);
    addLogLineCfg(buff);
  }
  
  if (err != aErrNone)
    addLogLineCfg("Stat retrieval failed");

  // re-enable relevant buttons
  // (recover is already disabled if stats has been called)
  osEnableWindowButton(kConfig, aGT_DUMP_CMD, aTrue);
  osEnableWindowButton(kConfig, aGT_STATS_CMD, aTrue);

}  // acpGarciaTool doStats method






/////////////////////////////////////////////////////////////////////
// acpGarciaTool doConfigMotoLink method

void acpGarciaTool::doConfigMotoLink(
  bool bRF
)
{
  aStreamRef newStream;
  aErr err = aErrNone;
  char* pPortName;

  // temporarily reconfigure network with 9600 baud link,
  // MOTO as router, and GP in serial relay mode

  aSettingFile_GetString(m_ioRef,
			 m_settings,
			 PORTNAMEKEY, 
			 &pPortName,
			 DEFAULTPORTNAME,
			 &err);

  // kill old serial stream and build default 9600 baud stream
  if (err == aErrNone)
    aStem_SetStream(m_stemRef, NULL, kStemModuleStream, &err);

  if (err == aErrNone)
    aStream_CreateSerial(m_ioRef, pPortName, 9600, 
    			 &newStream, &err);
  if (err == aErrNone)
    aStem_SetStream(m_stemRef, newStream, kStemModuleStream, &err);
    

  // reconfigure network
  if (err == aErrNone)
    err = aModuleVal_Set(m_stemRef,
			 aGARCIA_MOTO_ADDR,
			 aMODULE_VAL_ROUTER,
			 aGARCIA_MOTO_ADDR);
  if (err == aErrNone)
    err = aModuleVal_Set(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_ROUTER,
			 aGARCIA_MOTO_ADDR);


  // do different stuff for RF or Config mode
  if (bRF) {

    aSET_RF_STATUS_TEXT("Configuring...");

    osEnableWindowButton(kRF, aGT_RFLINK_CMD, aFalse);

    // this helps Mac display for some reason
    delayWithUpdates(7);

    if (err == aErrNone)
      err = aModuleVal_Set(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_RELAYFLAG,
			 1);

    if (err == aErrNone) {

      char cBaud = 0;

      // allow time to sync
      delayWithUpdates(1000);

      // get current GP baud rate (and verify response)
      err = aModuleVal_Get(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_SERBAUD,
			 &cBaud);

      if (err == aErrNone) {
        // set current GP baud rate
        osSetDropdownSel(kDropBoxRFBaud, cBaud);
      }
    }

    if (err == aErrNone) {
      aSET_RF_STATUS_TEXT("Moto Link Enabled");
      osEnableWindowButton(kRF, aGT_RFLINK_CMD, aFalse);
      osEnableWindowButton(kRF, aGT_RFREAD_CMD, aTrue);
      osEnableWindowButton(kRF, aGT_RFGPBAUD_CMD, aTrue);
      if (m_pRF->bCurrent)
        osEnableWindowButton(kRF, aGT_RFWRITE_CMD, aTrue);
    } else {
      osEnableWindowButton(kRF, aGT_RFLINK_CMD, aTrue);
      aSET_RF_STATUS_TEXT("Moto Link Failed!");
    }

  } else {

    // allow time to sync
    delayWithUpdates(1000);

    if ((err == aErrNone) && m_bLinkOkay) {
      addLogLineCfg("------------------------");
      addLogLineCfg("Moto link OK");
      osEnableWindowButton(kConfig, aGT_DUMP_CMD, aTrue);
      osEnableWindowButton(kConfig, aGT_RECOVER_CMD, aFalse);
      osEnableWindowButton(kConfig, aGT_STATS_CMD, aTrue);
      osEnableWindowButton(kConfig, aGT_MOTOLINK_CMD, aFalse);
      m_bMotoLink = true;
    } else {
      addLogLineCfg("------------------------");
      addLogLineCfg("Moto Link Failed!");
      addLogLineCfg("Check serial connection");
      addLogLineCfg("or try Recover.");
    }
  
  }

}  // acpGarciaTool doConfigMotoLink method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doRFRead method

void acpGarciaTool::doRFRead()
{
  aErr err = aErrNone;
  char status[10];
  int addr;

  // break up EEPROM read into small chunks
  // this prevents overflow when RF is at faster baud rate
  // than the 9600 baud default for the Moto board
  for (addr = 0; addr < 256; addr += aGT_CHUNKSIZE) {
    aStringFromInt(status, addr);
    aSET_RF_STATUS_TEXT(status);
    err = aGarciaRF_ReadEEPROM(m_pRF, addr, aGT_CHUNKSIZE);
    if (err != aErrNone) break;
  }

  // if successful then fill in data
  // and show RF module firmware version in status window
  if (err == aErrNone) {
    aGarciaRF_InitData(m_pRF);
    m_pRF->bCurrent = aTrue;
    osEnableWindowButton(kRF, aGT_RFWRITE_CMD, aTrue);
    aSET_RF_STATUS_TEXT(m_pRF->sVersion);
    osSetDropdownSel(kDropBoxRFChannel, m_pRF->cChannel);
    osSetDropdownSel(kDropBoxRFBaud, m_pRF->nBaud);
  } else {
    aSET_RF_STATUS_TEXT("RF read failed");
  }

}  // acpGarciaTool doRFRead method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doRFWrite method

void acpGarciaTool::doRFWrite()
{
  aErr err = aErrNone;

  // update RF data based on changes made
  m_pRF->nBaud = osGetDropdownSel(kDropBoxRFBaud);
  m_pRF->cChannel = (char)osGetDropdownSel(kDropBoxRFChannel);
  aGarciaRF_ApplyData(m_pRF);

  // do limited writes for now
  // (block to write baud stuff and pack len)
  // (block to write channel and tx attempts)
  if (err == aErrNone) {
    aSET_RF_STATUS_TEXT("Updating Baud");
    err = aGarciaRF_WriteEEPROM(m_pRF, aGARCIA_RF_BAUDH, 16);
  }
  if (err == aErrNone) {
    aSET_RF_STATUS_TEXT("Updating Channel");
    err = aGarciaRF_WriteEEPROM(m_pRF, aGARCIA_RF_CHANNEL, 2);
  }
  if (err == aErrNone) {
    aSET_RF_STATUS_TEXT("Updating System ID");
    err = aGarciaRF_WriteEEPROM(m_pRF, aGARCIA_RF_SYSTEMID, 8);
  }

  // must also update baud in GP
  if (err == aErrNone)
    err = doApplyGPBaud(aTrue, aFalse);

  if (err != aErrNone) {
    aSET_RF_STATUS_TEXT("RF Write failed");
  } else {
    aSET_RF_STATUS_TEXT("Success!  Reset robot.");
    osEnableWindowButton(kRF, aGT_RFREAD_CMD, aFalse);
    osEnableWindowButton(kRF, aGT_RFWRITE_CMD, aFalse);
    osEnableWindowButton(kRF, aGT_RFGPBAUD_CMD, aFalse);
  }

}  // acpGarciaTool doRFWrite method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doApplyGPBaud method

aErr acpGarciaTool::doApplyGPBaud(bool bReset, bool bVerbose)
{
  aErr err = aErrNone;

  if (bVerbose)
    aSET_RF_STATUS_TEXT("Updating GP Baud...");

  // update GP baud based on changes made
  m_pRF->nBaud = osGetDropdownSel(kDropBoxRFBaud);

  // must undo Moto link before
  // saving new baud rate to GP
  if (err == aErrNone)
    err = aModuleVal_Set(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_RELAYFLAG,
			 0);

  if (err == aErrNone)
    err = aModuleVal_Set(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_SERBAUD,
			 (char)m_pRF->nBaud);

  if (err == aErrNone)
    err = aModuleVal_Set(m_stemRef,
			 aGARCIA_GP_ADDR,
			 aMODULE_VAL_ROUTER,
			 aGARCIA_GP_ADDR);

  if (err == aErrNone)
    err = aModuleVal_Save(m_stemRef,
			 aGARCIA_GP_ADDR);

  // check for module reset
  if (bReset == aTrue) {
    if (err == aErrNone) {

      char data[aSTEMMAXPACKETBYTES];
      aPacketRef packet;
      char cBaud = -1;

      data[0] = cmdRESET;

      if (err == aErrNone)
        aPacket_Create(m_stemRef, aGARCIA_GP_ADDR, 1,
		       data, &packet, &err);

      if (err == aErrNone)
        aStem_SendPacket(m_stemRef,
			 packet, &err);

      // wait long enough for a reset (2sec)
      if (err == aErrNone) {
        delayWithUpdates(2000);
      }

      // reconfigure Moto link
      if (err == aErrNone)
        err = aModuleVal_Set(m_stemRef,
			     aGARCIA_GP_ADDR,
			     aMODULE_VAL_ROUTER,
			     aGARCIA_MOTO_ADDR);
      if (err == aErrNone)
        err = aModuleVal_Set(m_stemRef,
			     aGARCIA_GP_ADDR,
			     aMODULE_VAL_RELAYFLAG,
			     1);

      // allow time to sync
      if (err == aErrNone)
        delayWithUpdates(1000);

      // get current GP baud rate to verify success
      if (err == aErrNone)
        err = aModuleVal_Get(m_stemRef,
			     aGARCIA_GP_ADDR,
			     aMODULE_VAL_SERBAUD,
			     &cBaud);

      if (err == aErrNone) {
        if (cBaud != (char)osGetDropdownSel(kDropBoxRFBaud))
          err = aErrIO;
      }
    }
  }

  if (bVerbose) {
    if (err != aErrNone) {
      aSET_RF_STATUS_TEXT("GP baud change failed");
    } else {
      aSET_RF_STATUS_TEXT("GP baud change OK");
    }
  }

  return err;

}  // acpGarciaTool doApplyGPBaud method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool setGDText method

void acpGarciaTool::setGDText(
  aGDRef refGD,
  aRECT* pRect,
  const char* pText
)
{
  aErr err = aErrNone;
  aRECT r;

  aAssert(pRect);
  aAssert(refGD);

  r = (*pRect);
  r.x = 4;
  r.y = 0;

  if (err == aErrNone)
    aGD_StartDrawing(m_uiRef, refGD, &err);

  if (err == aErrNone)
    aGD_Erase(m_uiRef, refGD, &err);

  if (err == aErrNone)
    aGD_SetColor(m_uiRef, refGD, 0x000000, &err);

  if (err == aErrNone)
    aGD_Text(m_uiRef, refGD, &r, aUIALIGNLEFT, 
    	     12, pText, &err);

  if (err == aErrNone)
    aGD_EndDrawing(m_uiRef, refGD, &err);  
  
} // acpGarciaTool::setGDText method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doStopMotor method

aErr acpGarciaTool::doStopMotor(const int nMotor)
{
  aErr err = aErrNone;
  err = setMotionValue(nMotor, 0);
  if (err == aErrNone) {
    switch (nMotor) {
      case aGARCIA_MOTO_MOTOR_LEFT:
        osSetSlider(kSliderThrottleL, 0);
        break;
      case aGARCIA_MOTO_MOTOR_RIGHT:
        osSetSlider(kSliderThrottleR, 0);
        break;
    }
  }
  return err;
  
} // acpGarciaTool doStopMotor method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool setMotionValue method

aErr acpGarciaTool::setMotionValue(int nMotor, int nValue)
{
  aErr err = aErrNone;
  if (!m_nDampVelState) {
    // set speed via set point
    err = aMotion_SetValue(m_stemRef,
			   aGARCIA_MOTO_ADDR,
			   (unsigned char)nMotor,
			   (short)nValue);
  } else {
    // set speed via ramp param
    err = aMotion_SetRampParam(m_stemRef,
			       aGARCIA_MOTO_ADDR,
			       (unsigned char)nMotor,
			       aMOTION_RIVMAX,
			       (short)nValue);
  }
  return err;

} // setMotionValue



/////////////////////////////////////////////////////////////////////
// acpGarciaTool queueValueForIRTX method

void acpGarciaTool::queueValueForIRTX(int nValue)
{
  char buff[48];
  char cuff[8];

  if (m_nIRTXqueueCt < aGT_IRTX_QUEUE_SIZE) {
    aStringCopy(buff,"queued ");
    intToHexString(cuff, nValue);
    aStringCat(buff, cuff);
    addLogLineIR(buff);
    m_nIRTXqueue[m_nIRTXqueueCt] = nValue;
    m_nIRTXqueueCt++;
  } else {
    addLogLineIR("queue full!");
  }
  
} // acpGarciaTool::queueValueForIRTX method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool setDualRampParams method

aErr acpGarciaTool::setDualRampParams(int nIndex, int nValue)
{
  aErr err = aErrNone;
  err = aMotion_SetRampParam(m_stemRef,
			     aGARCIA_MOTO_ADDR,
			     aGARCIA_MOTO_MOTOR_RIGHT,
			     (unsigned char)nIndex,
			     (short)nValue);
  err = aMotion_SetRampParam(m_stemRef,
			     aGARCIA_MOTO_ADDR,
			     aGARCIA_MOTO_MOTOR_LEFT,
			     (unsigned char)nIndex,
			     (short)nValue);
  return err;

} // setDualRampParams



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doTestIRTX method

void acpGarciaTool::doTestIRTX()
{
  int i;

  // flush queue
  m_nIRTXqueueCt = 0;

  // queue some fake data
  for (i = 1; i < aGT_IRTX_QUEUE_SIZE; i++) {
    queueValueForIRTX(i + i * 256);
  }
 
} // acpGarciaTool doTestIRTX method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doSendQueueIRTX method

void acpGarciaTool::doSendQueueIRTX()
{
  int i;
  int n;
  aErr err = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];
  
  for (i = 0; i < m_nIRTXqueueCt; i++) {

    n = m_nIRTXqueue[i];
    data[0] = cmdIRP_XMIT;
    data[1] = aGARCIA_GP_DIRCOMM_TX;
    aUtil_StoreShort(&data[2], (short)n);

    if (err == aErrNone)
      aPacket_Create(m_stemRef, aGARCIA_GP_ADDR, 4,
		       data, &packet, &err);

    if (err == aErrNone)
        aStem_SendPacket(m_stemRef,
			 packet, &err);

    delayWithUpdates(250);
    addLogLineIR(".");
  }

  m_nIRTXqueueCt = 0;
  addLogLineIR("queue empty");
  addLogLineIR("-----------");

} // acpGarciaTool doSendQueueIRTX method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doEncZero method

void acpGarciaTool::doEncZero()
{
  aErr err = aErrNone;
  aPacketRef packet;
  char data[aSTEMMAXPACKETBYTES];
  
  data[0] = cmdMO_ENC32;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;

  data[1] = aGARCIA_MOTO_MOTOR_LEFT;
  if (err == aErrNone)
    aPacket_Create(m_stemRef, aGARCIA_MOTO_ADDR, 6,
		       data, &packet, &err);
  if (err == aErrNone)
      aStem_SendPacket(m_stemRef,
			 packet, &err);

  data[1] = aGARCIA_MOTO_MOTOR_RIGHT;
  if (err == aErrNone)
    aPacket_Create(m_stemRef, aGARCIA_MOTO_ADDR, 6,
		       data, &packet, &err);
  if (err == aErrNone)
      aStem_SendPacket(m_stemRef,
			 packet, &err);

  // immediately zero display
  aStringFromInt(data, 0);
  aSET_MO_ENC32L_TEXT(data);
  aStringFromInt(data, 0);
  aSET_MO_ENC32R_TEXT(data);
 
} // acpGarciaTool doEncZero method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool rangerEnable method

void acpGarciaTool::rangerEnable()
{
  // force the ranger enables all on (inverted logic)

  if (m_bFrontRangers)
    aDigital_WriteInt(m_stemRef,
  		      aGARCIA_GP_ADDR,
  		      aGARCIA_GP_DENABLE_FRONTRNG,
  		      0);

  if (m_bSideRangers)
    aDigital_WriteInt(m_stemRef,
  		      aGARCIA_GP_ADDR,
  		      aGARCIA_GP_DENABLE_SIDERNG,
  		      0);

  if (m_bRearRangers)
    aDigital_WriteInt(m_stemRef,
  		      aGARCIA_GP_ADDR,
  		      aGARCIA_GP_DENABLE_REARRNG,
  		      0);

  if (m_bDownRangers)
    aDigital_WriteInt(m_stemRef,
  		      aGARCIA_GP_ADDR,
  		      aGARCIA_GP_DENABLE_DOWNRNG,
  		      0);

} // acpGarciaTool rangerEnable method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doDampingControl method

void acpGarciaTool::doDampingControl()
{
  aErr err = aErrNone;

  // stop motors if switching to damping
  if (!m_nDampVelState) {
    doStopMotor(aGARCIA_MOTO_MOTOR_LEFT);
    doStopMotor(aGARCIA_MOTO_MOTOR_RIGHT);
  }

  // set new damp flag state
  m_nDampVelState = osGetCheckBox(kCheckBoxMoDamp);
  
  if (m_nDampVelState) {
    // enable damping
    err = setDualRampParams(aMOTION_RIACCT, m_nDampVelConst);
    err = setDualRampParams(aMOTION_RIFLAG, (1 << aMOTION_RAMPFLAG_TYPE));
    err = aMotion_RampEnable(m_stemRef,
			       aGARCIA_MOTO_ADDR,
			       1,
			       aMOTION_RXALL);
  } else {
    // disable damping
    err = aMotion_RampEnable(m_stemRef,
			       aGARCIA_MOTO_ADDR,
			       0,
			       aMOTION_RXALL);
    err = setDualRampParams(aMOTION_RIFLAG, 0);
    err = setDualRampParams(aMOTION_RIVMAX, 0);
  }

  // stop motors if damping is now disabled
  if (!m_nDampVelState) {
    doStopMotor(aGARCIA_MOTO_MOTOR_LEFT);
    doStopMotor(aGARCIA_MOTO_MOTOR_RIGHT);
  }

} // acpGarciaTool doDampingControl method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool setMotoAmpGraph method

void acpGarciaTool::setMotoAmpGraph(
  const int nMotor,
  const float fVoltage)
{
  aErr err = aErrNone;
  aRECT rIndicator = {
    0, 0, aGT_MO_GD_WIDTH, aGT_BAT_HEIGHT
  };
  aRECT rBounds;
  aGDRef gdRef = NULL;
  
  switch(nMotor) {
    case aGARCIA_MOTO_MOTOR_LEFT:
      rBounds = m_rMotoAmpL;
      gdRef = m_gdMotoAmpL;
      break;
    case aGARCIA_MOTO_MOTOR_RIGHT:
      rBounds = m_rMotoAmpR;
      gdRef = m_gdMotoAmpR;
      break;
  }
  
  aAssert(gdRef);

  if (err == aErrNone)
    aGD_StartDrawing(m_uiRef, gdRef, &err);

  if (err == aErrNone)
    aGD_Erase(m_uiRef, gdRef, &err);

  if (err == aErrNone)
    aGD_SetColor(m_uiRef, gdRef, 0xFF0000, &err);

  // create bar graph
  rIndicator.x += 1;
  rIndicator.y += 1;
  rIndicator.height -= 2;
  rIndicator.width = (aUIPixelType)(rIndicator.width * fVoltage * 3.3f);
  if (rIndicator.width > aGT_MO_GD_WIDTH)
    rIndicator.width = aGT_MO_GD_WIDTH;
  
  if (err == aErrNone)
    aGD_Rect(m_uiRef, gdRef, &rIndicator, aTrue, &err);

  if (err == aErrNone)
    aGD_EndDrawing(m_uiRef, gdRef, &err);
  
} // acpGarciaTool setMotoAmpGraph



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doApplyDampingConstant method

void acpGarciaTool::doApplyDampingConstant()
{
  int n = 0;
  char buff[80];
  osGetText(kParamMoDamp, 80, buff);
  aIntFromString(&n, buff);
  if (n < aGT_DAMPVEL_MIN) n = aGT_DAMPVEL_MIN;
  if (n > aGT_DAMPVEL_MAX) n = aGT_DAMPVEL_MAX;
  setDampingConstant(n);
  aStringFromInt(buff, n);
  osSetText(kParamMoDamp, buff);

} // acpGarciaTool doApplyDampingConstant method



/////////////////////////////////////////////////////////////////////
// acpGarciaTool doApplyIRTXData method

void acpGarciaTool::doApplyIRTXData()
{
  int n;
  char buff[80];
  osGetText(kParamIRTX, 80, buff);
  hexStringToInt(&n, buff);
  queueValueForIRTX(n);
  intToHexString(buff, n);
  osSetText(kParamIRTX, buff);

} // acpGarciaTool doApplyIRTXData method
