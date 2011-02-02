/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_apiPACKAGE.cpp	 	 		           //
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

#include "acpTag_apiPACKAGE.h"

#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// apiPACKAGE = 0x0014
/////////////////////////////////////////////////////////////////////


acpTag_apiPACKAGE::acpTag_apiPACKAGE() :
  acpPackageTag(aTAG_apiPACKAGE, NULL)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_apiPACKAGE::~acpTag_apiPACKAGE()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_apiPACKAGE::setData(
  const aShort nOwnerID,
  const aByte cDistanceUnit,
  const aByte cAngleUnit,
  const aByte cMassUnit
)
{
  setOwnerID(nOwnerID);
  m_distanceUnit = cDistanceUnit;
  m_angleUnit = cAngleUnit;
  m_massUnit = cMassUnit;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_apiPACKAGE::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_apiPACKAGE* pPkgData = new acpTag_apiPACKAGE();

  pPkgData->setOwnerID(acpShort(stream));
  pPkgData->m_distanceUnit = acpByte(stream);
  pPkgData->m_angleUnit = acpByte(stream);
  pPkgData->m_massUnit = acpByte(stream);

  return pPkgData;
}


/////////////////////////////////////////////////////////////////////

void acpTag_apiPACKAGE::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  acpByte cdist(m_distanceUnit);
  cdist.writeToStream(stream);

  acpByte cang(m_angleUnit);
  cang.writeToStream(stream);

  acpByte cmass(m_massUnit);
  cmass.writeToStream(stream);
}
