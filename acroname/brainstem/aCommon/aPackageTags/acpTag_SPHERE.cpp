/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_SPHERE.cpp	 	 		           //
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

#include "acpTag_SPHERE.h"

#include "acpShort.h"
#include "acpByte.h"
#include "acpFloat.h"



/////////////////////////////////////////////////////////////////////
// SPHERE = 0x0003
/////////////////////////////////////////////////////////////////////

acpTag_SPHERE::~acpTag_SPHERE()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_SPHERE::setData(
  const aShort nOwnerID,
  const acpVec3& color,
  const asReal radius
)
{
  setOwnerID(nOwnerID);
  m_color = color;
  m_radius = radius;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_SPHERE::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_SPHERE* pSphere = new acpTag_SPHERE();

  acpShort ownerID(stream);
  pSphere->setOwnerID(ownerID);

  aByte r = acpByte(stream);
  aByte g = acpByte(stream);
  aByte b = acpByte(stream);

  pSphere->m_color = acpVec3((float)((unsigned char)r / 254.0), 
  		    	     (float)((unsigned char)g / 254.0),
  		    	     (float)((unsigned char)b / 254.0));

  pSphere->m_radius = acpFloat(stream);

  return pSphere;
}


/////////////////////////////////////////////////////////////////////

void acpTag_SPHERE::writeData(
  aStreamRef stream
) const
{
  // the owner index
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  // the color
  acpByte r((aByte)(m_color.m_x * 254));
  r.writeToStream(stream);
  acpByte g((aByte)(m_color.m_y * 254));
  g.writeToStream(stream);
  acpByte b((aByte)(m_color.m_z * 254));
  b.writeToStream(stream);
  
  // the radius
  acpFloat radius(m_radius);
  radius.writeToStream(stream);
}
