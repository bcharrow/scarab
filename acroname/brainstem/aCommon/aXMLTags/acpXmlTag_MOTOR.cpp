/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_MOTOR.cpp	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of XML tag class.                   //
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

#include "acpXmlTag_MOTOR.h"

#include "acpTag_MOTOR.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

MOTOR::~MOTOR()
{
  // clear out the dynamics under us since the XML tree will 
  // clean them up
  while (m_dynamics.length())
    m_dynamics.removeHead();
}


/////////////////////////////////////////////////////////////////////

bool MOTOR::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "DYNAMIC")) {
    m_dynamics.add((DYNAMIC*)pTag);
    return true;
  } else if (!aStringCompare(pTagName, "AXIS"))
    return aEXASSIGN(m_pAxis, pTag);
  else if (!aStringCompare(pTagName, "TRANSLATION"))
    return aEXASSIGN(m_pTranslation, pTag);
  else if (!aStringCompare(pTagName, "NAME"))
    return aEXASSIGN(m_pName, pTag);
  else if (!aStringCompare(pTagName, "MOTORLINK"))
    return aEXASSIGN(m_pMotorLink, pTag);
  else if (!aStringCompare(pTagName, "TORQUE"))
    return aEXASSIGN(m_pTorque, pTag);
  else if (!aStringCompare(pTagName, "TORQUEMIN"))
    return aEXASSIGN(m_pTorqueMin, pTag);

  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

char* MOTOR::getObjectName()
{ 
  if (m_pName)
    return (char*)m_pName->m_name; 
  return NULL;
}


/////////////////////////////////////////////////////////////////////

void MOTOR::traverse()
{
  if (!m_dynamics.length())
    tagError("MOTOR must have a single DYNAMIC child");
  else {
    acpTag_MOTOR* pMotorTag = new acpTag_MOTOR();
    m_pImporter->addTag(this, pMotorTag);

    aShort ID = pMotorTag->getID();

    // need to step through each dynamic and assign it's 
    // owner
    // also need the notion of a grouping node
    // show that the child is owned by this motor

    aLISTITERATE(DYNAMIC, m_dynamics, pDynamic) {
      pDynamic->setOwnerID(ID);
      pDynamic->traverse();
    }

    acpString name;
    if (m_pName)
      name = m_pName->m_name;

    acpVec3 axis(1, 0, 0);
    if (m_pAxis)
      axis = acpVec3(m_pAxis->m_x, m_pAxis->m_y, m_pAxis->m_z);

    acpVec3 translation(0, 0, 0);
    if (m_pTranslation)
      translation = acpVec3(m_pTranslation->m_x, 
		            m_pTranslation->m_y, 
			    m_pTranslation->m_z);

    aFloat torque = 1.0f;
    if (m_pTorque)
      torque = *m_pTorque;
    
    aFloat torqueMin = 0.0f;
    if (m_pTorqueMin)
      torqueMin = *m_pTorqueMin;

    pMotorTag->setData(m_pImporter->m_nCurrentOwner, 
		       name, 
		       translation, 
		       axis,
		       torque,
		       torqueMin);

    // allow any motor links to build the tags for linkage joints
    if (m_pMotorLink) {
      m_pMotorLink->m_parentMotor = ID;
      m_pMotorLink->traverse();
    }
  }
}
