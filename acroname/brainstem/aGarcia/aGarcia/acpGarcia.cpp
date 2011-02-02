/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarcia.cpp                                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API library object.   //
//              This acts as a wrapper class to provide a clean,   //
//              and simple interface to the acpGarciaInternal      //
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

#include "acpGarcia.h"
#include "acpGarciaInternal.h"
#include "acpBehavior.h"

#include "acpXMLScript.h"


/////////////////////////////////////////////////////////////////////
// acpGarcia constructor
//

acpGarcia::acpGarcia() :
  acpObject("garcia", "garcia")
{
  m_pInternal = new acpGarciaInternal(this);

} // acpGarcia constructor



/////////////////////////////////////////////////////////////////////
// acpGarcia destructor
//

acpGarcia::~acpGarcia()
{
  delete m_pInternal;

} // acpGarcia destructor



/////////////////////////////////////////////////////////////////////
// acpGarcia numProperties method
//

int acpGarcia::numProperties () const
{
  return m_pInternal->numProperties();

} // acpGarcia numProperties



/////////////////////////////////////////////////////////////////////
// acpGarciaInternal enumProperties method
//

void acpGarcia::enumProperties (
  propertyEnumProc enumProc,
  void* vpRef
) const
{
  m_pInternal->enumProperties(enumProc, vpRef);

} // acpGarcia enumProperties



/////////////////////////////////////////////////////////////////////
// acpGarcia getPropertyIndex method
//

int acpGarcia::getPropertyIndex(
  const char* pPropertyName
) const
{
  return m_pInternal->getPropertyIndex(pPropertyName);
}



/////////////////////////////////////////////////////////////////////
// acpGarcia getValue method
//
char* acpGarcia::getPropertyName(
  const int nPropertyIndex) const
{
  return m_pInternal->getPropertyName(nPropertyIndex);
}



/////////////////////////////////////////////////////////////////////
// acpGarcia getPropertyFlags method
//

aPROPERTY_FLAGS	acpGarcia::getPropertyFlags(
  const int nPropertyIndex) const
{
  return m_pInternal->getPropertyFlags(nPropertyIndex);
}



/////////////////////////////////////////////////////////////////////
// acpGarcia getNamedValue method
//

acpValue* acpGarcia::getNamedValue (
  const char* pPropName
)
{
  return m_pInternal->getNamedValue(pPropName);

} // acpGarcia getNamedValue method



/////////////////////////////////////////////////////////////////////
// acpGarcia getValue method
//

acpValue* acpGarcia::getValue (
  const int nPropIndex
)
{
  return m_pInternal->getValue(nPropIndex);

} // acpGarcia getValue method



/////////////////////////////////////////////////////////////////////
// acpGarcia setNamedValue method
//

void acpGarcia::setNamedValue (
  const char* pPropName,
  const acpValue* pValue
)
{
  m_pInternal->setNamedValue(pPropName, pValue);

} // acpGarcia setNamedValue method



/////////////////////////////////////////////////////////////////////
// acpGarcia setValue method
//

void acpGarcia::setValue (
  const int nPropIndex,
  const acpValue* pValue
)
{
  m_pInternal->setValue(nPropIndex, pValue);

} // acpGarcia getValue method



/////////////////////////////////////////////////////////////////////
// acpGarcia numSubObjects method
//

int acpGarcia::numSubObjects () const
{
  return m_pInternal->numSubObjects();

} // acpGarcia numSubObjects



/////////////////////////////////////////////////////////////////////
// acpGarcia enumSubObjects method
//

void acpGarcia::enumSubObjects(
  aObjectEnumProc enumProc,
  void* vpRef,
  const char* pClassNameFilter
) const
{
  m_pInternal->enumSubObjects(enumProc, vpRef, pClassNameFilter);
}



/////////////////////////////////////////////////////////////////////
// acpGarcia getSubObject method
//

acpObject* acpGarcia::getSubObject (
  const char* pClassName,
  const char* pName
) const
{
  return m_pInternal->getNamedSubObject(pClassName, pName);

} // acpGarcia getSubObject



/////////////////////////////////////////////////////////////////////
// acpGarcia getSubObject method
//

acpObject* acpGarcia::getSubObject (
  const int nObjectIndex
) const
{
  return m_pInternal->getSubObject(nObjectIndex);

} // acpGarcia getSubObject


#if 0
/////////////////////////////////////////////////////////////////////

acpValue* acpGarcia::getBehaviorValue (
  aBehaviorRef behavior,
  const char* pPropName
)
{
  acpBehavior* pBehavior = (acpBehavior*)behavior;
  return pBehavior->getValue(pPropName);
}



/////////////////////////////////////////////////////////////////////

acpValue* acpGarcia::getBehaviorValue (
  aBehaviorRef behavior,
  const int nPropIndex
)
{
  acpBehavior* pBehavior = (acpBehavior*)behavior;
  return pBehavior->getValue(nPropIndex);
}



/////////////////////////////////////////////////////////////////////

aErr acpGarcia::readBehaviorValue(
  aBehaviorRef behavior,
  const int nPropIndex,
  aStreamRef source)
{
  acpBehavior* pBehavior = (acpBehavior*)behavior;
  return pBehavior->readValue(nPropIndex, source);
}



/////////////////////////////////////////////////////////////////////

void acpGarcia::setBehaviorValue (
  aBehaviorRef behavior,
  const char* pPropName,
  const acpValue* pValue
)
{
  acpBehavior* pBehavior = (acpBehavior*)behavior;
  pBehavior->setValue(pPropName, pValue);
}



/////////////////////////////////////////////////////////////////////

void acpGarcia::setBehaviorValue (
  aBehaviorRef behavior,
  const int nPropIndex,
  const acpValue* pValue
)
{
  acpBehavior* pBehavior = (acpBehavior*)behavior;  
  pBehavior->setValue(nPropIndex, pValue);
}
#endif


/////////////////////////////////////////////////////////////////////
// acpGarcia queueBehavior method
//

void acpGarcia::queueBehavior(
  acpObject* pBehavior
)
{
  aAssert(pBehavior);

  m_pInternal->queueBehavior((acpBehavior*)pBehavior);

} // acpGarcia queueBehavior




#if 0
/////////////////////////////////////////////////////////////////////
// acpGarcia isActive method
//

bool acpGarcia::isActive()
{
  return m_pInternal->isActive();

} // acpGarcia isActive


/////////////////////////////////////////////////////////////////////
// acpGarcia isIdle method
//

bool acpGarcia::isIdle()
{
  return m_pInternal->isIdle();

} // acpGarcia isIdle
#endif



int acpGarcia::handleCallbacks (
  const unsigned long nMSYield
)
{
  return m_pInternal->handleCallerMessages(nMSYield);

} // acpGarcia handleCallbacks



/////////////////////////////////////////////////////////////////////
// acpGarcia statusString method
//

char* acpGarcia::statusString (
  const short nStatus,
  char* pText,
  unsigned int nMaxChars
)
{
  return m_pInternal->statusString(nStatus, pText, nMaxChars);

} // acpGarcia statusString method

#if 0
/////////////////////////////////////////////////////////////////////
// acpGarcia ticksPerUnitDistance method
//

unsigned int acpGarcia::ticksPerUnitDistance()
{
  return m_pInternal->ticksPerUnitDistance();

} // acpGarcia ticksPerUnitDistance method



/////////////////////////////////////////////////////////////////////
// acpGarcia ticksPerUnitRotation method
//

unsigned int acpGarcia::ticksPerUnitRotation()
{ 
  return m_pInternal->ticksPerUnitRotation(); 

} // acpGarcia ticksPerUnitRotation method
#endif


/////////////////////////////////////////////////////////////////////
// acpGarcia createBehavior method
//

acpObject* acpGarcia::createNamedBehavior (
  const char* pPrimitiveName,
  const char* pBehaviorName
)
{
  return m_pInternal->createNamedBehavior(pPrimitiveName, 
  				     pBehaviorName);

} // acpGarcia createBehavior method



/////////////////////////////////////////////////////////////////////
// acpGarcia createBehavior method
//

acpObject* acpGarcia::createBehavior (
  const int nPrimitiveIndex,
  const char* pBehaviorName
)
{
  return m_pInternal->createBehavior(nPrimitiveIndex, 
  				     pBehaviorName);

} // acpGarcia createBehavior method



#if 0
/////////////////////////////////////////////////////////////////////
// acpGarcia destroyBehavior method
//

void acpGarcia::destroyBehavior (
  aBehaviorRef behavior
)
{
  acpBehavior* pBehavior = (acpBehavior*)behavior;

  delete pBehavior;

} // acpGarcia destroyBehavior method
#endif



/////////////////////////////////////////////////////////////////////
// acpGarcia flushQueuedBehaviors method
//

void acpGarcia::flushQueuedBehaviors()
{
  m_pInternal->flushQueuedBehaviors();

} // acpGarcia flushQueuedBehaviors method
