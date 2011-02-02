/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_PROPERTY.cpp	 	 		           //
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

#include "acpProperty.h"
#include "acpTag_PROPERTY.h"

#include "acpStringIO.h"
#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// PROPERTY = 0x000D
/////////////////////////////////////////////////////////////////////


acpTag_PROPERTY::acpTag_PROPERTY() :
  acpPackageTag(aTAG_PROPERTY, NULL)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_PROPERTY::~acpTag_PROPERTY()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_PROPERTY::setData(
  const aShort nOwnerID,
  const char* pName,
  const char* pLinkName,
  const aInt32 nFlags,
  const aByte cUnitType,
  const aByte cUnitCode,
  const char* pDefault,
  const char* pDescription
)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  m_linkname = pLinkName;
  m_flags = nFlags;
  m_unittype = cUnitType;
  m_unitcode = cUnitCode;
  m_default = pDefault;
  m_description = pDescription;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_PROPERTY::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_PROPERTY* pProp = new acpTag_PROPERTY();

  pProp->setOwnerID(acpShort(stream));
  pProp->m_name = acpStringIO(stream);
  pProp->m_linkname = acpStringIO(stream);
  pProp->m_flags = acpInt32(stream);
  pProp->m_unittype = acpByte(stream);
  pProp->m_unitcode = acpByte(stream);
  pProp->m_default = acpStringIO(stream);
  pProp->m_description = acpStringIO(stream);

  return pProp;
}


/////////////////////////////////////////////////////////////////////

void acpTag_PROPERTY::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  acpStringIO name(m_name);
  name.writeToStream(stream);

  acpStringIO linkname(m_linkname);
  linkname.writeToStream(stream);

  acpInt32 nflags(m_flags);
  nflags.writeToStream(stream);

  acpByte ctype(m_unittype);
  ctype.writeToStream(stream);

  acpByte ccode(m_unitcode);
  ccode.writeToStream(stream);

  acpStringIO sdefault(m_default);
  sdefault.writeToStream(stream);

  acpStringIO sdescription(m_description);
  sdescription.writeToStream(stream);
}
