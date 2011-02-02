/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_PLANE.cpp	 	 		           //
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

#include "acpTag_PLANE.h"



/////////////////////////////////////////////////////////////////////
// PLANE = 0x0007
/////////////////////////////////////////////////////////////////////

acpTag_PLANE::acpTag_PLANE() :
  acpPackageTag(aTAG_PLANE, (void*)1),
  m_color(0,0,0),
  m_normal(0,0,0),
  m_height(0.0f)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_PLANE::~acpTag_PLANE()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_PLANE::setData(
  const aShort nOwnerID,
  acpVec3& color,
  acpVec3& normal,
  aReal height
)
{
  setOwnerID(nOwnerID);
  m_color = color;
  m_normal = normal;
  m_height = (float)height;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_PLANE::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_PLANE* pPlane = new acpTag_PLANE();

  pPlane->m_color = acpVec3(stream);
  pPlane->m_normal = acpVec3(stream);
  pPlane->m_height = acpFloat(stream);

  return pPlane;
}


/////////////////////////////////////////////////////////////////////

void acpTag_PLANE::writeData(
  aStreamRef stream
) const
{
  acpVec3 color(m_color);
  color.writeToStream(stream);

  acpVec3 normal(m_normal);
  normal.writeToStream(stream);

  acpFloat height(m_height);
  height.writeToStream(stream);
}
