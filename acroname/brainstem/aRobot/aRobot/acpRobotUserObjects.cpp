/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotUserObjects.cpp                                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the user object template         //
//              and user object instance classes.                  //
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

#include "acpRobotUserObjects.h"



/////////////////////////////////////////////////////////////////////
// acpRobotBaseObject constructor
//

acpRobotBaseObject::acpRobotBaseObject (
  acpRobotInternal* pcRobotInternal,
  acpTag_TEMPLATE* pTemplate
) :
  acpObject("template", pTemplate->m_name),
  m_pcRobotInternal(pcRobotInternal),
  m_nUserPropCt(0),
  m_nBufferSize(pTemplate->m_nBufferSize)
{
  // add dummy dataptr to make property sets
  // identical between base object and user object
  acpValue dataptr((void*)NULL);
  addProperty(new acpProperty(aROBOT_PROPNAME_DATAPTR, 
  			      aPROPERTY_FLAG_VOIDPTR
  			      | aPROPERTY_FLAG_READ),
  			      &dataptr);
}

acpRobotBaseObject::~acpRobotBaseObject()
{
}



/////////////////////////////////////////////////////////////////////
// acpRobotBaseObject addBaseProperty method
//

int acpRobotBaseObject::addBaseProperty(
  acpRobotUserProperty* pProperty,
  acpValue* pDefault
)
{
  m_nUserPropCt++;

  return acpObject::addProperty(pProperty, pDefault); 

} // addObjectProperty method




/////////////////////////////////////////////////////////////////////
// acpRobotUserObject constructor
//

acpRobotUserObject::acpRobotUserObject (
  acpRobotInternal* pcRobotInternal,
  acpTag_USEROBJECT* pObjTag
) :
  acpObject("user_object", pObjTag->m_name),
  m_pcRobotInternal(pcRobotInternal),
  m_pcTemplate(NULL),
  m_pData(NULL),
  m_nDataBytes(0),
  m_nLinkID(aROBOT_APITEA_NOTFOUND)
{
  acpValue dataptr((void*)NULL);
  addProperty(new acpProperty(aROBOT_PROPNAME_DATAPTR, 
  			      aPROPERTY_FLAG_VOIDPTR
  			      | aPROPERTY_FLAG_READ),
  			      &dataptr);
}

acpRobotUserObject::~acpRobotUserObject()
{
  if (m_pData)
    aMemFree(m_pData);
}

void  acpRobotUserObject::setTemplate(
  acpRobotBaseObject* pTemplate)
{
  aAssert(pTemplate);
  m_pcTemplate = pTemplate;

  // free any old storage
  if (m_pData)
    aMemFree(m_pData);

  // allocate new storage
  m_nDataBytes = m_pcTemplate->getBufferSize();
  if (m_nDataBytes)
    m_pData = (char*)aMemAlloc(sizeof(char) * m_nDataBytes);
  if (m_pData) {
    aBZero(m_pData, m_nDataBytes);
    acpValue dataptr((void*)m_pData);
    setNamedValue(aROBOT_PROPNAME_DATAPTR, &dataptr);
  }

  // add place holder properties
  // that mirror the template properties
  // (only way things will work with robot agent)
  int numStart =
    m_pcTemplate->numProperties() - m_pcTemplate->getUserPropCt();
  for (int i = numStart; i < m_pcTemplate->numProperties(); i++) {
    acpRobotUserProperty* pProp = 
      (acpRobotUserProperty*) m_pcTemplate->getProperty(i);
    aPROPERTY_FLAGS flags = pProp->getTypeFlags();
    flags |= aPROPERTY_FLAG_USERBIT;
    addProperty(new acpProperty(pProp->getName(), flags));
  }
}

void  acpRobotUserObject::initialize(
  const int nPropIndex
)
{
  aAssert(m_pcTemplate);

  // get template property and set its link ID
  // based on the link ID of this user object
//  int k = m_pcTemplate->getPropertyIndex(pPropName);
  acpRobotUserProperty* pProp =
    (acpRobotUserProperty*)m_pcTemplate->getProperty(nPropIndex);
  pProp->m_nLinkID = m_nLinkID;

  // only one property API TEA program runs at a time
  // so we set the VM manager's property data pointer now
  m_pcRobotInternal->
    m_pcVMM[aROBOT_APITEA_PROPPROCID]->
      setPropDataPtr(m_pData, m_nDataBytes);
}

acpValue* acpRobotUserObject::getNamedValue(
  const char* pPropName
)
{
  int i = getPropertyIndex(pPropName);
  return getValue(i);
}

acpValue* acpRobotUserObject::getValue(
  const int nPropIndex
)
{
  aPROPERTY_FLAGS flags =
    getProperty(nPropIndex)->getTypeFlags();

  if (flags & aPROPERTY_FLAG_USERBIT) {

    // this is a user property
    // which is accessed via the template
    initialize(nPropIndex);
    return m_pcTemplate->getValue(nPropIndex);

  } else {

    // this is a base class property
    return acpObject::getValue(nPropIndex);
  }
}

void acpRobotUserObject::setNamedValue(
  const char* pPropName,
  const acpValue* pValue
)
{
  int i = getPropertyIndex(pPropName);
  setValue(i, pValue);
}

void acpRobotUserObject::setValue(
  const int nPropIndex,
  const acpValue* pValue
)
{
  aPROPERTY_FLAGS flags =
    getProperty(nPropIndex)->getTypeFlags();

  if (flags & aPROPERTY_FLAG_USERBIT) {

    // this is a user property
    // which is accessed via the template
    initialize(nPropIndex);
    m_pcTemplate->setValue(nPropIndex, pValue);

  } else {

    // this is a base class property
    acpObject::setValue(nPropIndex, pValue);
  }
}
