/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaServo.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API servo object.     //
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

#include "acpGarciaServo.h"
#include "aGarciaProperties.h"
#include "acpGarciaInternal.h"
#include "acpCallbackMessage.h"

#include "aCmd.tea"
#include "aGarciaDefs.tea"
#include "aServo.h"



/////////////////////////////////////////////////////////////////////

acpGarciaServo::acpGarciaServo (
  const char* pName,
  const char cModule,
  const char cIndex,
  acpGarciaInternal* pcGarciaInternal
) :
  acpObject("servo", pName),
  m_cModule(cModule),
  m_cIndex(cIndex),
  m_pcGarciaInternal(pcGarciaInternal)
{
  addProperty(new acpGarciaServoPosAbsProperty(this));
  addProperty(new acpGarciaServoPosRelProperty(this));

  // these use cmdSRV_CFG
  addProperty(new acpGarciaServoEnableProperty(this));
  addProperty(new acpGarciaServoInvertProperty(this));
  addProperty(new acpGarciaServoDisabledStateProperty(this));
  addProperty(new acpGarciaServoSpeedProperty(this));

  // these use cmdSRV_LMT
  addProperty(new acpGarciaServoOffsetProperty(this));
  addProperty(new acpGarciaServoRangeProperty(this));
}



/////////////////////////////////////////////////////////////////////

acpGarciaServo::~acpGarciaServo()
{
} // acpGarciaServo destructor


/////////////////////////////////////////////////////////////////////

void acpGarciaServoProperty::sendServoPacket (
  acpGarciaServo* pcGarciaServo,
  char cCmd,
  char data1)
{
  char data[aSTEMMAXPACKETBYTES];
  data[0] = cCmd;
  data[1] = m_pcGarciaServo->m_cIndex;
  data[2] = data1;
  m_pcGarciaServo->m_pcGarciaInternal->sendStemPacket (
    (unsigned char)m_pcGarciaServo->m_cModule, 3, data);
}

void acpGarciaServoProperty::sendServoPacket (
  acpGarciaServo* pcGarciaServo,
  char cCmd,
  char data1,
  char data2)
{
  char data[aSTEMMAXPACKETBYTES];
  data[0] = cCmd;
  data[1] = m_pcGarciaServo->m_cIndex;
  data[2] = data1;
  data[3] = data2;
  m_pcGarciaServo->m_pcGarciaInternal->sendStemPacket (
    (unsigned char)m_pcGarciaServo->m_cModule, 4, data);
}


/////////////////////////////////////////////////////////////////////

aErr acpGarciaServo::writeToStream(const aStreamRef stream) const
{
  return aErrNone;

} // writeToStream method




/////////////////////////////////////////////////////////////////////

acpGarciaServoProperty::acpGarciaServoProperty (
  acpGarciaServo* pcGarciaServo,
  const char* pName,
  aPROPERTY_FLAGS flags
) :
  acpProperty(pName, flags),
  m_pcGarciaServo(pcGarciaServo)
{
}



/////////////////////////////////////////////////////////////////////
// position absolute
/////////////////////////////////////////////////////////////////////

acpGarciaServoPosAbsProperty::acpGarciaServoPosAbsProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_POSITION_ABS,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}

void acpGarciaServoPosAbsProperty::setValue (
  const acpValue* pValue)
{
  int nValue = (int)(255.0f * pValue->getFloatVal());
  if (nValue > 255) nValue = 255;
  if (nValue < 0) nValue = 0;
  sendServoPacket(m_pcGarciaServo, cmdSRV_ABS, char(nValue));
}

void acpGarciaServoPosAbsProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set(getServoPositionValue());
}



/////////////////////////////////////////////////////////////////////
// position relative
/////////////////////////////////////////////////////////////////////

acpGarciaServoPosRelProperty::acpGarciaServoPosRelProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_POSITION_REL,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}

void acpGarciaServoPosRelProperty::setValue (
  const acpValue* pValue)
{
  int nSign = 0;
  int nValue = (int)(255.0f * pValue->getFloatVal());
  if (nValue > 255) nValue = 255;
  if (nValue < -255) nValue = -255;
  if (nValue < 0) {
    nSign = 1;
    nValue = -nValue;
  }
  sendServoPacket(m_pcGarciaServo, cmdSRV_REL, (char)nValue, (char)nSign);
}

void acpGarciaServoPosRelProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set(0.0f);
}



/////////////////////////////////////////////////////////////////////
// enable flag
/////////////////////////////////////////////////////////////////////

acpGarciaServoEnableProperty::acpGarciaServoEnableProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_ENABLE_FLAG,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_BOOL))
{
}

void acpGarciaServoEnableProperty::setValue (
  const acpValue* pValue)
{
  bool benable = pValue->getBoolVal();
  char config = (char)getServoConfigValue();
  config &= (char)(~aSERVO_ENA);
  if (benable) config |= (char)aSERVO_ENA;
  sendServoPacket(m_pcGarciaServo, cmdSRV_CFG, config);
}

void acpGarciaServoEnableProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  bool bFlag = ((getServoConfigValue() & aSERVO_ENA) != 0);
  pValue->set(bFlag);
}



/////////////////////////////////////////////////////////////////////
// invert flag
/////////////////////////////////////////////////////////////////////

acpGarciaServoInvertProperty::acpGarciaServoInvertProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_INVERT_FLAG,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_BOOL))
{
}

void acpGarciaServoInvertProperty::setValue (
  const acpValue* pValue)
{
  bool binvert = pValue->getBoolVal();
  char config = (char)getServoConfigValue();
  config &= (char)(~aSERVO_INV);
  if (binvert) config |= (char)aSERVO_INV;
  sendServoPacket(m_pcGarciaServo, cmdSRV_CFG, config);
}

void acpGarciaServoInvertProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  bool bFlag = ((getServoConfigValue() & aSERVO_INV) != 0);
  pValue->set(bFlag);
}



/////////////////////////////////////////////////////////////////////
// disabled state
/////////////////////////////////////////////////////////////////////

acpGarciaServoDisabledStateProperty::acpGarciaServoDisabledStateProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_DISABLEDSTATE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_BOOL))
{
}

void acpGarciaServoDisabledStateProperty::setValue (
  const acpValue* pValue)
{
  bool bstate = pValue->getBoolVal();
  char config = (char)getServoConfigValue();
  config &= (char)(~aSERVO_DSTA);
  if (bstate) config |= (char)aSERVO_DSTA;
  sendServoPacket(m_pcGarciaServo, cmdSRV_CFG, config);
}

void acpGarciaServoDisabledStateProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  bool bFlag = ((getServoConfigValue() & aSERVO_DSTA) != 0);
  pValue->set(bFlag);
}



/////////////////////////////////////////////////////////////////////
// speed
/////////////////////////////////////////////////////////////////////

acpGarciaServoSpeedProperty::acpGarciaServoSpeedProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_SPEED,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaServoSpeedProperty::setValue (
  const acpValue* pValue)
{
  char cspeed = (char)pValue->getIntVal();
  char config = (char)getServoConfigValue();
  config &= ~aSERVO_SPEEDMASK;
  config |= cspeed;
  sendServoPacket(m_pcGarciaServo, cmdSRV_CFG, config);
}

void acpGarciaServoSpeedProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  int nspeed = getServoConfigValue() & aSERVO_SPEEDMASK;
  pValue->set(nspeed);
}



/////////////////////////////////////////////////////////////////////
// offset
/////////////////////////////////////////////////////////////////////

acpGarciaServoOffsetProperty::acpGarciaServoOffsetProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_OFFSET,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaServoOffsetProperty::setValue (
  const acpValue* pValue)
{
  int nlimits = getServoLimitsValue();
  char coffset = (char)pValue->getIntVal();
  char crange = (char)(nlimits & 0xFF);
  sendServoPacket(m_pcGarciaServo, cmdSRV_LMT, coffset, crange);
}

void acpGarciaServoOffsetProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  int noffset = getServoLimitsValue();
  noffset = (noffset & 0xFF00) >> 8;
  pValue->set(noffset);
}



/////////////////////////////////////////////////////////////////////
// offset
/////////////////////////////////////////////////////////////////////

acpGarciaServoRangeProperty::acpGarciaServoRangeProperty (
  				  acpGarciaServo* pcGarciaServo) :
  acpGarciaServoProperty(pcGarciaServo,
  		      aGSP_RANGE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaServoRangeProperty::setValue (
  const acpValue* pValue)
{
  int nlimits = getServoLimitsValue();
  char crange = (char)pValue->getIntVal();
  char coffset = (char)((nlimits & 0xFF00) >> 8);
  sendServoPacket(m_pcGarciaServo, cmdSRV_LMT, coffset, crange);
}

void acpGarciaServoRangeProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  int nrange = getServoLimitsValue();
  nrange &= 0x00FF;
  pValue->set(nrange);
}
