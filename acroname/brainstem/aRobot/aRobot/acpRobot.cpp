/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobot.cpp                                              //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API library object.    //
//              This acts as a wrapper class to provide a clean,   //
//              and simple interface to the acpRobotInternal       //
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

#include "acpRobot.h"
#include "acpProperty.h"
#include "acpRobotInternal.h"
#include "acpRobotBehavior.h"

#include "acpRobotXMLReader.h"


/////////////////////////////////////////////////////////////////////
// acpRobot constructor
//

acpRobot::acpRobot(
  const char* pName // = NULL
) :
  acpObject("robot", pName)
{
  m_pInternal = new acpRobotInternal(this, pName);

} // acpRobot constructor



/////////////////////////////////////////////////////////////////////
// acpRobot destructor
//

acpRobot::~acpRobot()
{
  delete m_pInternal;

} // acpRobot destructor



/////////////////////////////////////////////////////////////////////
// acpRobot addProperty method
//

void acpRobot::addProperty(
  acpProperty* pProperty,
  acpValue* pDefault // = NULL
)
{
  printf("adding property named %s to %X\n", 
	 pProperty->getName(), (int)(long)this);
  m_pInternal->addProperty(pProperty, pDefault);

} // acpRobot addProperty



/////////////////////////////////////////////////////////////////////
// acpRobot numProperties method
//

int acpRobot::numProperties () const
{
  return m_pInternal->numProperties();

} // acpRobot numProperties



/////////////////////////////////////////////////////////////////////
// acpRobotInternal enumProperties method
//

void acpRobot::enumProperties (
  propertyEnumProc enumProc,
  void* vpRef
) const
{
  m_pInternal->enumProperties(enumProc, vpRef);

} // acpRobot enumProperties


/////////////////////////////////////////////////////////////////////
// acpRobot getProperty method
//

acpProperty* acpRobot::getProperty(
  const int nPropertyIndex) const
{
  return m_pInternal->getProperty(nPropertyIndex);
}


/////////////////////////////////////////////////////////////////////
// acpRobot getPropertyIndex method
//

int acpRobot::getPropertyIndex(
  const char* pPropertyName
) const
{
  return m_pInternal->getPropertyIndex(pPropertyName);
}



/////////////////////////////////////////////////////////////////////
// acpRobot getValue method
//
char* acpRobot::getPropertyName(
  const int nPropertyIndex) const
{
  return m_pInternal->getPropertyName(nPropertyIndex);
}



/////////////////////////////////////////////////////////////////////
// acpRobot getPropertyFlags method
//

aPROPERTY_FLAGS	acpRobot::getPropertyFlags(
  const int nPropertyIndex) const
{
  return m_pInternal->getPropertyFlags(nPropertyIndex);
}



/////////////////////////////////////////////////////////////////////
// acpRobot getNamedValue method
//

acpValue* acpRobot::getNamedValue (
  const char* pPropName
)
{
  return m_pInternal->getNamedValue(pPropName);

} // acpRobot getNamedValue method



/////////////////////////////////////////////////////////////////////
// acpRobot getValue method
//

acpValue* acpRobot::getValue (
  const int nPropIndex
)
{
  return m_pInternal->getValue(nPropIndex);

} // acpRobot getValue method



/////////////////////////////////////////////////////////////////////
// acpRobot setNamedValue method
//

void acpRobot::setNamedValue (
  const char* pPropName,
  const acpValue* pValue
)
{
  m_pInternal->setNamedValue(pPropName, pValue);

} // acpRobot setNamedValue method



/////////////////////////////////////////////////////////////////////
// acpRobot setValue method
//

void acpRobot::setValue (
  const int nPropIndex,
  const acpValue* pValue
)
{
  m_pInternal->setValue(nPropIndex, pValue);

} // acpRobot getValue method



/////////////////////////////////////////////////////////////////////
// acpRobot numSubObjects method
//

int acpRobot::numSubObjects () const
{
  return m_pInternal->numSubObjects();

} // acpRobot numSubObjects



/////////////////////////////////////////////////////////////////////
// acpRobot enumSubObjects method
//

void acpRobot::enumSubObjects(
  aObjectEnumProc enumProc,
  void* vpRef,
  const char* pClassNameFilter
) const
{
  m_pInternal->enumSubObjects(enumProc, vpRef, pClassNameFilter);
}



/////////////////////////////////////////////////////////////////////
// acpRobot getSubObject method
//

acpObject* acpRobot::getSubObject (
  const char* pClassName,
  const char* pName
) const
{
  return m_pInternal->getNamedSubObject(pClassName, pName);

} // acpRobot getSubObject



/////////////////////////////////////////////////////////////////////
// acpRobot getSubObject method
//

acpObject* acpRobot::getSubObject (
  const int nObjectIndex
) const
{
  return m_pInternal->getSubObject(nObjectIndex);

} // acpRobot getSubObject



/////////////////////////////////////////////////////////////////////
// acpRobot queueBehavior method
//

void acpRobot::queueBehavior(
  acpObject* pBehavior
)
{
  aAssert(pBehavior);

  m_pInternal->queueBehavior((acpRobotBehavior*)pBehavior);

} // acpRobot queueBehavior


/////////////////////////////////////////////////////////////////////
// acpRobot handleCallbacks method
//

int acpRobot::handleCallbacks (
  const unsigned long nMSYield
)
{
  return m_pInternal->handleCallerMessages(nMSYield);

} // acpRobot handleCallbacks



/////////////////////////////////////////////////////////////////////
// acpRobot statusString method
//

char* acpRobot::statusString (
  const short nStatus,
  char* pText,
  unsigned int nMaxChars
)
{
  return m_pInternal->statusString(nStatus, pText, nMaxChars);

} // acpRobot statusString method



/////////////////////////////////////////////////////////////////////
// acpRobot createBehavior method
//

acpObject* acpRobot::createNamedBehavior (
  const char* pPrimitiveName,
  const char* pBehaviorName
)
{
  return m_pInternal->createNamedBehavior(pPrimitiveName, 
  				     pBehaviorName);

} // acpRobot createBehavior method



/////////////////////////////////////////////////////////////////////
// acpRobot createBehavior method
//

acpObject* acpRobot::createBehavior (
  const int nPrimitiveIndex,
  const char* pBehaviorName
)
{
  return m_pInternal->createBehavior(nPrimitiveIndex, 
  				     pBehaviorName);

} // acpRobot createBehavior method



/////////////////////////////////////////////////////////////////////
// acpRobot flushQueuedBehaviors method
//

void acpRobot::flushQueuedBehaviors()
{
  m_pInternal->flushQueuedBehaviors();

} // acpRobot flushQueuedBehaviors method
