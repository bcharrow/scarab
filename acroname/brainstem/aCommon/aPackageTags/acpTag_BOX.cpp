/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_BOX.cpp		 	 		           //
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

#include "acpTag_BOX.h"

#include "acpShort.h"



/////////////////////////////////////////////////////////////////////
// BOX = 0x0008
/////////////////////////////////////////////////////////////////////


acpTag_BOX::acpTag_BOX() :
  acpPackageTag(aTAG_BOX, (void*)1),
  m_color(0,0,0),
  m_dimensions(0,0,0)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_BOX::~acpTag_BOX()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_BOX::setData(
  const aShort nOwnerID,
  acpVec3& color,
  acpVec3& dimensions
)
{
  setOwnerID(nOwnerID);
  m_color = color;
  m_dimensions = dimensions;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_BOX::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_BOX* pBox = new acpTag_BOX();

  pBox->setOwnerID(acpShort(stream));
  pBox->m_color = acpVec3(stream);
  pBox->m_dimensions = acpVec3(stream);
  
  return pBox;
}


/////////////////////////////////////////////////////////////////////

void acpTag_BOX::writeData(
  aStreamRef stream
) const
{
  // the owner index
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  acpVec3 color(m_color);
  color.writeToStream(stream);

  acpVec3 dimensions(m_dimensions);
  dimensions.writeToStream(stream);
}
