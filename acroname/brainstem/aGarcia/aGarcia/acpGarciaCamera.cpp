/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaCamera.cpp                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API camera object.    //
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

#include "acpGarciaCamera.h"
#include "aGarciaProperties.h"
#include "acpGarciaInternal.h"
#include "acpCallbackMessage.h"

#include "aCmd.tea"
#include "aGarciaDefs.tea"
#include "aServo.h"



/////////////////////////////////////////////////////////////////////

acpGarciaCamera::acpGarciaCamera (
  const char* pName,
  const unsigned char cModule,
  acpGarciaInternal* pcGarciaInternal
) :
  acpObject("camera", pName),
  m_cModule(cModule),
  m_pcGarciaInternal(pcGarciaInternal)
{
  addProperty(new acpGarciaCameraPanProperty(this));
  addProperty(new acpGarciaCameraTiltProperty(this));
}



/////////////////////////////////////////////////////////////////////

acpGarciaCamera::~acpGarciaCamera()
{
} // acpGarciaCamera destructor


/////////////////////////////////////////////////////////////////////

void acpGarciaCameraProperty::sendPacket (
  acpGarciaCamera* pcGarciaCamera,
  unsigned char cModule,
  unsigned char cLen,
  char* data)
{
  m_pcGarciaCamera->m_pcGarciaInternal->sendStemPacket (
    cModule, cLen, data);
}



/////////////////////////////////////////////////////////////////////

aErr acpGarciaCamera::writeToStream(const aStreamRef stream) const
{
  return aErrNone;

} // writeToStream method




/////////////////////////////////////////////////////////////////////

acpGarciaCameraProperty::acpGarciaCameraProperty (
  acpGarciaCamera* pcGarciaCamera,
  const char* pName,
  aPROPERTY_FLAGS flags
) :
  acpProperty(pName, flags),
  m_pcGarciaCamera(pcGarciaCamera)
{
}



/////////////////////////////////////////////////////////////////////
// pan
/////////////////////////////////////////////////////////////////////

acpGarciaCameraPanProperty::acpGarciaCameraPanProperty (
  				  acpGarciaCamera* pcGarciaCamera) :
  acpGarciaCameraProperty(pcGarciaCamera,
  		      aGCP_PAN,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}

void acpGarciaCameraPanProperty::setValue (
  const acpValue* pValue)
{
  char data[aSTEMMAXPACKETBYTES];
  float f = pValue->getFloatVal();
  if (f > 1.0f) f = 1.0f;
  if (f < -1.0f) f = -1.0f;
  int nValue = (int)(128 + (127.0f * f));
  data[0] = cmdSRV_ABS;
  data[1] = aGCP_PAN_INDEX;
  data[2] = (char)nValue;
  sendPacket(m_pcGarciaCamera, m_pcGarciaCamera->getModule(), 3, data);
}

void acpGarciaCameraPanProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((getServoPositionValue(aGCP_PAN_INDEX) * 2.0f) - 1.0f);
}



/////////////////////////////////////////////////////////////////////
// tilt
/////////////////////////////////////////////////////////////////////

acpGarciaCameraTiltProperty::acpGarciaCameraTiltProperty (
  				  acpGarciaCamera* pcGarciaCamera) :
  acpGarciaCameraProperty(pcGarciaCamera,
  		      aGCP_TILT,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}

void acpGarciaCameraTiltProperty::setValue (
  const acpValue* pValue)
{

  char data[aSTEMMAXPACKETBYTES];
  float f = pValue->getFloatVal();
  if (f > 1.0f) f = 1.0f;
  if (f < -1.0f) f = -1.0f;
  int nValue = (int)(128 + (127.0f * f));
  data[0] = cmdSRV_ABS;
  data[1] = aGCP_TILT_INDEX;
  data[2] = (char)nValue;
  sendPacket(m_pcGarciaCamera, m_pcGarciaCamera->getModule(), 3, data);
}

void acpGarciaCameraTiltProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((getServoPositionValue(aGCP_TILT_INDEX) * 2.0f) - 1.0f);
}
