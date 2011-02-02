/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotObject.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotShell client object.       //
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

#include "acpRobotShell.h"
#include "acpRobotObject.h"


/////////////////////////////////////////////////////////////////////

acpRobotObject::~acpRobotObject() 
{
  // clean out the properties
  acpRobotProperty* pProperty;
  while ((pProperty = m_properties.removeHead()))
    delete pProperty;

  // clean out the children
  acpRobotObject* pChild;
  while ((pChild = m_children.removeHead())) 
    delete pChild;
}


/////////////////////////////////////////////////////////////////////

void acpRobotObject::ensurePropertiesEnumerated() 
{
  if (!m_pShell->m_pCurrent->m_bPropertiesEnumerated) {
    m_pShell->sendChar('P');
    m_pShell->sendInt(m_ID, true);
    int nProperties = m_pShell->nextShort();
    for (int i = 0; i < nProperties; i++) {
      acpString name;
      m_pShell->nextString(&name);
      int typeFlags = m_pShell->nextInt();
      acpRobotProperty* pProperty = 
      	new acpRobotProperty(name, i, typeFlags);
      m_pShell->m_pCurrent->addProperty(pProperty);
    }
    m_pShell->m_pCurrent->m_bPropertiesEnumerated = true;
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotObject::ensureObjectsEnumerated()
{
  if (!m_pShell->m_pCurrent->m_bObjectsEnumerated) {
    m_pShell->sendChar('O');
    m_pShell->sendInt(m_ID, true);
    int nObjects = m_pShell->nextShort();
    for (int i = 0; i < nObjects; i++) {
      acpString className;
      m_pShell->nextString(&className);
      acpString name;
      m_pShell->nextString(&name);
      int id = m_pShell->nextInt();
      acpRobotObject* pObject = 
      	new acpRobotObject(m_pShell, className, name);
      pObject->setID(id);
      m_pShell->m_pCurrent->addChild(pObject);
    }
    m_pShell->m_pCurrent->m_bObjectsEnumerated = true;
  }
}


/////////////////////////////////////////////////////////////////////

acpRobotProperty* acpRobotObject::getProperty(
  const char* pName
) 
{
  ensurePropertiesEnumerated();

  aLISTITERATE(acpRobotProperty, m_properties, pProperty) {
    if (!aStringCompare(pName, pProperty->getName())) {
      return pProperty;
    }
  }
  return NULL;
}



/////////////////////////////////////////////////////////////////////

acpRobotObject* acpRobotObject::getObject(
  const char* pName
) 
{
  ensureObjectsEnumerated();

  aLISTITERATE(acpRobotObject, m_children, pObject) {
    if (!aStringCompare(pName, pObject->getName())) {
      return pObject;
    }
  }
  return NULL;
}
