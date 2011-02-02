/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_CYLINDER.cpp	 	 		           //
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

#include "acpTag_CYLINDER.h"

#include "acpShort.h"



/////////////////////////////////////////////////////////////////////
// CYLINDER = 0x0009
/////////////////////////////////////////////////////////////////////

acpTag_CYLINDER::acpTag_CYLINDER() :
  acpPackageTag(aTAG_CYLINDER, (void*)true),
  m_color(0,0,0),
  m_radius(0.0f),
  m_length(0.0f)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_CYLINDER::~acpTag_CYLINDER()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_CYLINDER::setData(
  const aShort nOwnerID,
  acpVec3& color,
  aReal radius,
  aReal length
)
{
  setOwnerID(nOwnerID);
  m_color = color;
  m_radius = radius;
  m_length = length;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_CYLINDER::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_CYLINDER* pCylinder = new acpTag_CYLINDER();

  pCylinder->setOwnerID(acpShort(stream));
  pCylinder->m_color = acpVec3(stream);
  pCylinder->m_radius = acpFloat(stream);
  pCylinder->m_length = acpFloat(stream);

  return pCylinder;
}


/////////////////////////////////////////////////////////////////////

void acpTag_CYLINDER::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  acpVec3 color(m_color);
  color.writeToStream(stream);

  acpFloat radius(m_radius);
  radius.writeToStream(stream);

  acpFloat length(m_length);
  length.writeToStream(stream);
}
