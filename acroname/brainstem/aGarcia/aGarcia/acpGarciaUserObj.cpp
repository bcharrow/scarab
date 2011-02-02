/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaUserObj.cpp                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API UserObj object.   //
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

#include "acpGarciaUserObj.h"
#include "aGarciaProperties.h"
#include "acpGarciaInternal.h"
#include "acpCallbackMessage.h"

#include "aCmd.tea"
#include "aGarciaDefs.tea"



/////////////////////////////////////////////////////////////////////

acpGarciaUserObj::acpGarciaUserObj (
  const char* pName,
  acpGarciaInternal* pcGarciaInternal
) :
  acpObject("userobj", pName),
  m_cModule(0),
  m_cSize(0),
  m_cFilterByte(0),
  m_ulTimeout(0),
  m_pcGarciaInternal(pcGarciaInternal)
{
  addProperty(new acpGarciaUserObjModuleProperty(this));
  addProperty(new acpGarciaUserObjSizeProperty(this));
  addProperty(new acpGarciaUserObjFilterByteProperty(this));
  addProperty(new acpGarciaUserObjTimeoutProperty(this));
  addProperty(new acpGarciaUserObjDataPtrProperty(this));
  addProperty(new acpGarciaUserObjResultProperty(this));
}



/////////////////////////////////////////////////////////////////////

acpGarciaUserObj::~acpGarciaUserObj()
{
} // acpGarciaUserObj destructor

/////////////////////////////////////////////////////////////////////

void acpGarciaUserObjProperty::sendUserObjPacket()
{
  m_pcGarciaUserObj->m_pcGarciaInternal->sendStemPacket (
    (unsigned char)m_pcGarciaUserObj->m_cModule,
    (unsigned char)m_pcGarciaUserObj->m_cSize,
    m_pcGarciaUserObj->m_pData);
}

/////////////////////////////////////////////////////////////////////

int acpGarciaUserObjProperty::doUserObjAction()
{
  int r;
  r = m_pcGarciaUserObj->m_pcGarciaInternal->getUserObjValue(
        (unsigned char)m_pcGarciaUserObj->m_cModule,
        (unsigned char)m_pcGarciaUserObj->m_cSize,
        (unsigned char)m_pcGarciaUserObj->m_cFilterByte,
        m_pcGarciaUserObj->m_ulTimeout,
        m_pcGarciaUserObj->m_pData);
  return r;
}

/////////////////////////////////////////////////////////////////////

aErr acpGarciaUserObj::writeToStream(const aStreamRef stream) const
{
  return aErrNone;

} // writeToStream method




/////////////////////////////////////////////////////////////////////

acpGarciaUserObjProperty::acpGarciaUserObjProperty (
  acpGarciaUserObj* pcGarciaUserObj,
  const char* pName,
  aPROPERTY_FLAGS flags
) :
  acpProperty(pName, flags),
  m_pcGarciaUserObj(pcGarciaUserObj)
{
}



/////////////////////////////////////////////////////////////////////
// module
/////////////////////////////////////////////////////////////////////

acpGarciaUserObjModuleProperty::acpGarciaUserObjModuleProperty (
  				  acpGarciaUserObj* pcGarciaUserObj) :
  acpGarciaUserObjProperty(pcGarciaUserObj,
  		      aGUP_MODULE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaUserObjModuleProperty::setValue (
  const acpValue* pValue)
{
  m_pcGarciaUserObj->m_cModule = (char)pValue->getIntVal();
}

void acpGarciaUserObjModuleProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set(m_pcGarciaUserObj->m_cModule);
}



/////////////////////////////////////////////////////////////////////
// size
/////////////////////////////////////////////////////////////////////

acpGarciaUserObjSizeProperty::acpGarciaUserObjSizeProperty (
  				  acpGarciaUserObj* pcGarciaUserObj) :
  acpGarciaUserObjProperty(pcGarciaUserObj,
  		      aGUP_SIZE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaUserObjSizeProperty::setValue (
  const acpValue* pValue)
{
  m_pcGarciaUserObj->m_cSize = (char)pValue->getIntVal();
}

void acpGarciaUserObjSizeProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set(m_pcGarciaUserObj->m_cSize);
}



/////////////////////////////////////////////////////////////////////
// filter-byte
/////////////////////////////////////////////////////////////////////

acpGarciaUserObjFilterByteProperty::acpGarciaUserObjFilterByteProperty (
  				  acpGarciaUserObj* pcGarciaUserObj) :
  acpGarciaUserObjProperty(pcGarciaUserObj,
  		      aGUP_FILTER_BYTE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaUserObjFilterByteProperty::setValue (
  const acpValue* pValue)
{
  m_pcGarciaUserObj->m_cFilterByte = (char)pValue->getIntVal();
}

void acpGarciaUserObjFilterByteProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set(m_pcGarciaUserObj->m_cFilterByte);
}



/////////////////////////////////////////////////////////////////////
// timeout
/////////////////////////////////////////////////////////////////////

acpGarciaUserObjTimeoutProperty::acpGarciaUserObjTimeoutProperty (
  				  acpGarciaUserObj* pcGarciaUserObj) :
  acpGarciaUserObjProperty(pcGarciaUserObj,
  		      aGUP_TIMEOUT,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaUserObjTimeoutProperty::setValue (
  const acpValue* pValue)
{
  m_pcGarciaUserObj->m_ulTimeout = (unsigned long)pValue->getIntVal();
}

void acpGarciaUserObjTimeoutProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((int)m_pcGarciaUserObj->m_ulTimeout);
}



/////////////////////////////////////////////////////////////////////
// data-ptr
/////////////////////////////////////////////////////////////////////

acpGarciaUserObjDataPtrProperty::acpGarciaUserObjDataPtrProperty (
  				  acpGarciaUserObj* pcGarciaUserObj) :
  acpGarciaUserObjProperty(pcGarciaUserObj,
  		      aGUP_DATA_PTR,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_VOIDPTR))
{
}

void acpGarciaUserObjDataPtrProperty::setValue (
  const acpValue* pValue)
{
  m_pcGarciaUserObj->m_pData = (char*)pValue->getVoidPtrVal();
}

void acpGarciaUserObjDataPtrProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((void*)m_pcGarciaUserObj->m_pData);
}



/////////////////////////////////////////////////////////////////////
// result
/////////////////////////////////////////////////////////////////////

acpGarciaUserObjResultProperty::acpGarciaUserObjResultProperty (
  				  acpGarciaUserObj* pcGarciaUserObj) :
  acpGarciaUserObjProperty(pcGarciaUserObj,
  		      aGUP_RESULT,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}

void acpGarciaUserObjResultProperty::setValue (
  const acpValue* pValue)
{
}

void acpGarciaUserObjResultProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue)
{
  int r;
  r = doUserObjAction();
  pValue->set(r);
}
