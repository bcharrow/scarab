/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaSonar.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API Sonar object.     //
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

#include "acpGarciaSonar.h"
#include "aGarciaProperties.h"
#include "acpGarciaInternal.h"
#include "acpCallbackMessage.h"

#include "aCmd.tea"
#include "aGarciaDefs.tea"
#include "aSRF08.h"



/////////////////////////////////////////////////////////////////////

acpGarciaSonar::acpGarciaSonar (
  const char* pName,
  const unsigned char cModule,
  const unsigned char ucAddr,
  acpGarciaInternal* pcGarciaInternal
) :
  acpObject("sonar", pName),
  m_cModule(cModule),
  m_cSRF08addr(ucAddr),
  m_nEchoCount(1),
  m_pcGarciaInternal(pcGarciaInternal)
{
  addProperty(new acpGarciaSonarAddressProperty(this));
  addProperty(new acpGarciaSonarRangeProperty(this));
  addProperty(new acpGarciaSonarLightValueProperty(this));
  addProperty(new acpGarciaSonarRangeBufferProperty(this));
  addProperty(new acpGarciaSonarEchoCountProperty(this));
}



/////////////////////////////////////////////////////////////////////

acpGarciaSonar::~acpGarciaSonar()
{
} // acpGarciaSonar destructor



/////////////////////////////////////////////////////////////////////

aErr acpGarciaSonar::writeToStream(const aStreamRef stream) const
{
  return aErrNone;

} // writeToStream method



/////////////////////////////////////////////////////////////////////

acpGarciaSonarProperty::acpGarciaSonarProperty (
  acpGarciaSonar* pcGarciaSonar,
  const char* pName,
  aPROPERTY_FLAGS flags
) :
  acpProperty(pName, flags),
  m_pcGarciaSonar(pcGarciaSonar)
{
}

int acpGarciaSonarProperty::getSonarRangeValue(
  unsigned char units
)
{
  int r;
  r = m_pcGarciaSonar->m_pcGarciaInternal->getSonarRangeValue(
        m_pcGarciaSonar->m_cModule,
        m_pcGarciaSonar->m_cSRF08addr,
        units,
        m_pcGarciaSonar->m_nEchoCount,
        m_pcGarciaSonar->m_pRangeBuff);

  return r;
}

int acpGarciaSonarProperty::getSonarLightValue() const
{
  int r;
  r = m_pcGarciaSonar->m_pcGarciaInternal->getSonarLightValue(
        m_pcGarciaSonar->m_cModule,
        m_pcGarciaSonar->m_cSRF08addr);
  return r;
}

int acpGarciaSonarProperty::getDistanceUnitType() const
{
  return m_pcGarciaSonar->m_pcGarciaInternal->distanceUnitType();
}



/////////////////////////////////////////////////////////////////////
// address
/////////////////////////////////////////////////////////////////////

acpGarciaSonarAddressProperty::acpGarciaSonarAddressProperty (
  				  acpGarciaSonar* pcGarciaSonar) :
  acpGarciaSonarProperty(pcGarciaSonar,
  		      aGRP_ADDRESS,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaSonarAddressProperty::setValue (
  const acpValue* pValue)
{
  int nValue = pValue->getIntVal();
  if (nValue > 0xFE) nValue = 0xFE;
  if (nValue < 0xE0) nValue = 0xE0;
  m_pcGarciaSonar->m_cSRF08addr = (unsigned char)nValue;
}

void acpGarciaSonarAddressProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((int)m_pcGarciaSonar->m_cSRF08addr);
}



/////////////////////////////////////////////////////////////////////
// range
/////////////////////////////////////////////////////////////////////

acpGarciaSonarRangeProperty::acpGarciaSonarRangeProperty (
  				  acpGarciaSonar* pcGarciaSonar) :
  acpGarciaSonarProperty(pcGarciaSonar,
  		      aGRP_RANGE_INCH,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}

void acpGarciaSonarRangeProperty::setValue (
  const acpValue* pValue)
{
}

void acpGarciaSonarRangeProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  float r = (float)getSonarRangeValue(aSRF08_MS);

  // sometimes we get dropouts
  if (r < 0.0f) r = 0.0f;
  
  switch (getDistanceUnitType()) {
    case aGARCIA_DISTANCE_METERS:
      r = r * aGRP_VSOUND_MPERS / 2000000.0f;
      break;
    case aGARCIA_DISTANCE_FEET:
      r = r * aGRP_VSOUND_FPERS / 2000000.0f;
      break;
    case aGARCIA_DISTANCE_INCHES:
      r = r * aGRP_VSOUND_IPERS / 2000000.0f;
      break;
  }
  pValue->set(r);
}



/////////////////////////////////////////////////////////////////////
// light-value
/////////////////////////////////////////////////////////////////////

acpGarciaSonarLightValueProperty::acpGarciaSonarLightValueProperty (
  				  acpGarciaSonar* pcGarciaSonar) :
  acpGarciaSonarProperty(pcGarciaSonar,
  		      aGRP_LIGHT_VALUE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}

void acpGarciaSonarLightValueProperty::setValue (
  const acpValue* pValue)
{
}

void acpGarciaSonarLightValueProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  float r = (float)getSonarLightValue();
  r = r / 255.0f;
  pValue->set(r);
}



/////////////////////////////////////////////////////////////////////
// range-buffer
/////////////////////////////////////////////////////////////////////

acpGarciaSonarRangeBufferProperty::acpGarciaSonarRangeBufferProperty (
  				  acpGarciaSonar* pcGarciaSonar) :
  acpGarciaSonarProperty(pcGarciaSonar,
  		      aGRP_RANGE_BUFFER,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_VOIDPTR))
{
}

void acpGarciaSonarRangeBufferProperty::setValue (
  const acpValue* pValue)
{
}

void acpGarciaSonarRangeBufferProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((void*)m_pcGarciaSonar->m_pRangeBuff);
}



/////////////////////////////////////////////////////////////////////
// echo-count
/////////////////////////////////////////////////////////////////////

acpGarciaSonarEchoCountProperty::acpGarciaSonarEchoCountProperty (
  				  acpGarciaSonar* pcGarciaSonar) :
  acpGarciaSonarProperty(pcGarciaSonar,
  		      aGRP_ECHO_COUNT,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaSonarEchoCountProperty::setValue (
  const acpValue* pValue)
{
  m_pcGarciaSonar->m_nEchoCount = pValue->getIntVal();
}

void acpGarciaSonarEchoCountProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set(m_pcGarciaSonar->m_nEchoCount);
}
