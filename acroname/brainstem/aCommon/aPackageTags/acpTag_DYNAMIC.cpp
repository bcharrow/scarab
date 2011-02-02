/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_DYNAMIC.cpp	 	 		           //
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

#include "acpTag_DYNAMIC.h"

#include "acpShort.h"



/////////////////////////////////////////////////////////////////////
// DYNAMIC = 0x0005
/////////////////////////////////////////////////////////////////////

acpTag_DYNAMIC::~acpTag_DYNAMIC()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_DYNAMIC::setData(
  const aShort nOwnerID,
  const char* pName,
  const float fMass,
  const float fFriction,
  const float fSoftERP,
  const float fSoftCFM,
  const float fBounce,
  const acpVec3& position,
  const acpMatrix3& rotation)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  m_mass = fMass;
  m_friction = fFriction;
  m_soft_erp = fSoftERP;
  m_soft_cfm = fSoftCFM;
  m_bounce = fBounce;
  m_position = position;
  m_rotation = rotation;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_DYNAMIC::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_DYNAMIC* pDynamic = new acpTag_DYNAMIC();

  pDynamic->setOwnerID(acpShort(stream));

  pDynamic->m_name = acpStringIO(stream);

  pDynamic->m_mass = acpFloat(stream);

  pDynamic->m_friction = acpFloat(stream);
  pDynamic->m_soft_erp = acpFloat(stream);
  pDynamic->m_soft_cfm = acpFloat(stream);
  pDynamic->m_bounce = acpFloat(stream);

  pDynamic->m_position = acpVec3(stream);

  pDynamic->m_rotation = acpMatrix3(stream);

  return pDynamic;
}


/////////////////////////////////////////////////////////////////////

void acpTag_DYNAMIC::writeData(
  aStreamRef stream
) const
{
  // the owner index
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  // the name
  m_name.writeToStream(stream);

  // the mass
  m_mass.writeToStream(stream);

  // the contact parameters
  m_friction.writeToStream(stream);
  m_soft_erp.writeToStream(stream);
  m_soft_cfm.writeToStream(stream);
  m_bounce.writeToStream(stream);

  // the position
  m_position.writeToStream(stream);
  m_rotation.writeToStream(stream);
}
