/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_MOTOR.cpp	 	 		           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Package tag class.				   //
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

#include "acpTag_MOTOR.h"

#include "acpShort.h"




/////////////////////////////////////////////////////////////////////
// MOTOR = 0x000B
/////////////////////////////////////////////////////////////////////


acpTag_MOTOR::acpTag_MOTOR() :
  acpPackageTag(aTAG_MOTOR, (void*)1),
  m_translation(),
  m_axis(),
  m_torque(1.0f),
  m_torqueMin(0.0f)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_MOTOR::~acpTag_MOTOR()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_MOTOR::setData(
  const aShort nOwnerID,
  const char* pName,
  const acpVec3& translation,
  const acpVec3& axis,
  const aFloat torque,
  const aFloat torqueMin  
)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  m_translation = translation;
  m_axis = axis;
  m_torque = torque;
  m_torqueMin = torqueMin;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_MOTOR::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_MOTOR* pMotor = new acpTag_MOTOR();

  pMotor->setOwnerID(acpShort(stream));
  pMotor->m_name = acpStringIO(stream);
  pMotor->m_translation = acpVec3(stream);
  pMotor->m_axis = acpVec3(stream);
  pMotor->m_torque = acpFloat(stream);
  pMotor->m_torqueMin = acpFloat(stream);

  return pMotor;
}


/////////////////////////////////////////////////////////////////////

void acpTag_MOTOR::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);
  m_name.writeToStream(stream);
  m_translation.writeToStream(stream);
  m_axis.writeToStream(stream);
  m_torque.writeToStream(stream);
  m_torqueMin.writeToStream(stream);
}
