/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_INITIALIZER.cpp	 	 		           //
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

#include "acpTag_INITIALIZER.h"

#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// INITIALIZER = 0x0013
/////////////////////////////////////////////////////////////////////


acpTag_INITIALIZER::acpTag_INITIALIZER() :
  acpPackageTag(aTAG_INITIALIZER, NULL)
{
}


/////////////////////////////////////////////////////////////////////

acpTag_INITIALIZER::~acpTag_INITIALIZER()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_INITIALIZER::setData(
  const aShort nOwnerID,
  const char* pName,
  const char* pSvalue
)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  m_svalue = pSvalue;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_INITIALIZER::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_INITIALIZER* pInitTag = new acpTag_INITIALIZER();

  pInitTag->setOwnerID(acpShort(stream));
  pInitTag->m_name = acpStringIO(stream);
  pInitTag->m_svalue = acpStringIO(stream);

  return pInitTag;
}


/////////////////////////////////////////////////////////////////////

void acpTag_INITIALIZER::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);
  acpStringIO name(m_name);
  name.writeToStream(stream);
  acpStringIO svalue(m_svalue);
  svalue.writeToStream(stream);
}
