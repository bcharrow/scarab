/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpObject.cpp		  		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of Pure Virtual Base Property       //
//              Object Class Definition.                           //
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

#include "acpObject.h"



/////////////////////////////////////////////////////////////////////
// acpObject constructor
//

acpObject::acpObject(
  const char* pClassName,
  const char* pName // = NULL
) :
  m_nIndex(0)
{
  m_pSubObjects = new acpList<acpObject>;
  m_pValues = new acpList<acpValue>;
  m_pProperties = new acpList<acpProperty>;

  // class names are required
  acpValue classname(pClassName);
  m_nClassPropertyIndex = addProperty(new acpProperty("classname", 
  			      	      (aPROPERTY_FLAG_STRING 
			       	       | aPROPERTY_FLAG_READ)), 
			      	      &classname);

  // if name is not specified, use an empty string for the name
  acpValue name((pName) ? pName : "");
  m_nNamePropertyIndex = addProperty(new acpProperty("name", 
  			      	     (aPROPERTY_FLAG_STRING 
			       	      | aPROPERTY_FLAG_WRITE 
			       	      | aPROPERTY_FLAG_READ)), 
			      	     &name);

} // acpObject constructor



/////////////////////////////////////////////////////////////////////
// acpObject destructor
//

acpObject::~acpObject()
{
  delete m_pProperties;
  delete m_pValues;
  delete m_pSubObjects;

} // acpObject destructor



/////////////////////////////////////////////////////////////////////
// acpObject numSubObjects method
//

int acpObject::numSubObjects() const
{
  aAssert(m_pSubObjects);
  return (*m_pSubObjects).length();
}



/////////////////////////////////////////////////////////////////////
// acpObject enumSubObjects method
//

void acpObject::enumSubObjects(
  aObjectEnumProc enumProc,
  void* vpRef,
  const char* pClassNameFilter // = NULL
) const
{
  if (!enumProc)
    return;

  acpListIterator<acpObject> objects(*m_pSubObjects);
  acpObject* pObject = NULL;

  while ((pObject = objects.next())) {

    // if there was a class filter and we don't match it, skip
    // this object
    if (pClassNameFilter
        && aStringCompare(pObject->getValue(m_nClassPropertyIndex)->getStringVal(),
        		  pClassNameFilter))
      continue;

    aErr enumErr = enumProc(*pObject,
    			    vpRef);
    if (enumErr != aErrNone)
      return;
  }

} // enumSubObjects method



/////////////////////////////////////////////////////////////////////
// acpObject getNamedSubObject method
//
acpObject* acpObject::getNamedSubObject (
  const char* pClassName,
  const char* pName
) const
{
  acpListIterator<acpObject> objects(*m_pSubObjects);
  acpObject* pObject = NULL;

  while ((pObject = objects.next())) {
    acpValue* pvClass = pObject->getValue(m_nClassPropertyIndex);
    aAssert(pvClass);
    if (!aStringCompare(pClassName, pvClass->getStringVal())) {
      acpValue* pvName = pObject->getValue(m_nNamePropertyIndex);
      aAssert(pvName);
      const char* pObjectName = pvName->getStringVal();
      if (!aStringCompare(pName, pObjectName))
        break;
    }
  }
  
  return pObject;

} // getNamedSubObject method



/////////////////////////////////////////////////////////////////////
// acpObject getSubObject method
//

acpObject* acpObject::getSubObject (
  const int nObjectIndex
) const
{
  if (nObjectIndex < m_pSubObjects->length())
    return (*m_pSubObjects)[nObjectIndex];

  return NULL;
}


/////////////////////////////////////////////////////////////////////
// acpObject numProperties method
//

int acpObject::numProperties() const
{
  return m_pProperties->length();
}



/////////////////////////////////////////////////////////////////////
// acpObject enumProperties method
//

void acpObject::enumProperties (
  propertyEnumProc enumProc,
  void* vpRef
) const
{
  if (!enumProc)
    return;
 
  acpListIterator<acpProperty> properties(*m_pProperties);
  acpProperty* pProperty = NULL;

  int i = 0;
  while ((pProperty = properties.next())) {
    aErr enumErr = enumProc(pProperty->getName(),
    			    i++, pProperty->getTypeFlags(),
    			    vpRef);
    if (enumErr != aErrNone)
      return;
  }

} // acpObject enumProperties



/////////////////////////////////////////////////////////////////////
// acpObject getProperty method
//

acpProperty* acpObject::getProperty(
  const int nPropertyIndex) const
{
  return (*m_pProperties)[nPropertyIndex];
}



/////////////////////////////////////////////////////////////////////
// acpObject getPropertyIndex method
//

int acpObject::getPropertyIndex(
  const char* pPropertyName
) const
{
  acpListIterator<acpProperty> iterator(*m_pProperties);  
  acpProperty* pProperty = NULL;

  int i = 0;
  while ((pProperty = iterator.next())) {
    if (!aStringCompare(pPropertyName, pProperty->getName())) {
      return i;
    }
    i++;
  }

  return -1;
  
} // getPropertyIndex



/////////////////////////////////////////////////////////////////////
// acpObject getNamedValue method
//

acpValue* acpObject::getNamedValue (
  const char* pPropName
)
{
  return getValue(getPropertyIndex(pPropName));

} // acpObject getNamedValue method



/////////////////////////////////////////////////////////////////////
// acpObject getValue method
//

acpValue* acpObject::getValue (
  const int nPropIndex
)
{
  acpValue* pVal = NULL;

  // sanity check this, they should always be equal
  aAssert(m_pValues->length() == m_pProperties->length());

  // let the property manage getting the value
  if ((nPropIndex >= 0) && (nPropIndex < m_pValues->length())) {
    pVal = (*m_pValues)[nPropIndex];
    (*m_pProperties)[nPropIndex]->getValue(this, pVal);
  }

  return(pVal);

} // acpObject getValue method



/////////////////////////////////////////////////////////////////////
// acpObject setNamedValue method
//

void acpObject::setNamedValue (
  const char* pPropertyName,
  const acpValue* pValue
)
{
  int index = getPropertyIndex(pPropertyName);
  setValue(index, pValue);

} // acpObject setNamedValue method



/////////////////////////////////////////////////////////////////////
// acpObject setValue method
//

void acpObject::setValue(
  const int nPropIndex,
  const acpValue* pValue
)
{

  if ((nPropIndex >= 0) && (nPropIndex < m_pValues->length())) {
    acpValue* pVal = (*m_pValues)[nPropIndex];
    pVal->set(pValue);
    (*m_pProperties)[nPropIndex]->setValue(pVal);
  }

} // acpObject setValue method



/////////////////////////////////////////////////////////////////////
// acpObject getPropertyName method
//

char* acpObject::getPropertyName(
  const int nPropertyIndex) const
{

  // show invalid flags if out of range
  if ((nPropertyIndex < 0) 
      || (m_pProperties->length() <= nPropertyIndex))
    return NULL;

  return (*m_pProperties)[nPropertyIndex]->getName();

} // acpObject getPropertyName method



/////////////////////////////////////////////////////////////////////
// acpObject getPropertyFlags method
//

aPROPERTY_FLAGS	acpObject::getPropertyFlags(
  const int nPropertyIndex) const
{
  // show invalid flags if out of range
  if ((nPropertyIndex < 0) 
      || (m_pProperties->length() <= nPropertyIndex))
    return 0xFF;

  return (*m_pProperties)[nPropertyIndex]->getTypeFlags();

} // acpObject getPropertyFlags method




/////////////////////////////////////////////////////////////////////
// acpObject addProperty method
//

int acpObject::addProperty(
  acpProperty* pProperty,
  acpValue* pDefault // = NULL
)
{
  pProperty->setIndex(m_pProperties->length());

  m_pProperties->add(pProperty);

  // add a default empty value to the list
  acpValue* pDefVal;
  if (pDefault) 
    pDefVal = new acpValue(pDefault);
  else
    pDefVal = new acpValue;

  m_pValues->add(pDefVal);

  // should always have the same length since we use one to index
  // the other
  aAssert(m_pProperties->length() == m_pValues->length());
 
  return pProperty->m_nIndex;

} // addProperty method



/////////////////////////////////////////////////////////////////////
// acpObject addSubObject method
//

void acpObject::addSubObject(
  acpObject* pObject
)
{
  pObject->m_nIndex = m_pSubObjects->length();
  m_pSubObjects->add(pObject);

} // addSubObject method



/////////////////////////////////////////////////////////////////////
// acpObject readValue method
//

aErr acpObject::readValue(
  const int nPropIndex,
  aStreamRef stream
)
{
  aErr err = aErrNotFound;

  if ((nPropIndex >= 0) 
      && (nPropIndex < m_pProperties->length())) {
    acpValue* pValue = (*m_pValues)[nPropIndex];
    aAssert(pValue);
    acpProperty* pProperty = (*m_pProperties)[nPropIndex];
    err = pProperty->readValue(stream, pValue);
  }

  return err;

} // acpObject readValue method

