/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_USEROBJECT.cpp	 	 		           //
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

#include "acpTag_USEROBJECT.h"

#include "acpStringIO.h"
#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// USEROBJECT = 0x0012
/////////////////////////////////////////////////////////////////////


acpTag_USEROBJECT::acpTag_USEROBJECT() :
  acpPackageTag(aTAG_USEROBJECT, NULL)
{
}


/////////////////////////////////////////////////////////////////////

acpTag_USEROBJECT::~acpTag_USEROBJECT()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_USEROBJECT::setData(
  const aShort nOwnerID,
  const char* pName,
  const char* pLinkName,
  const char* pObjTagName,
  const int nInitCt
)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  m_linkname = pLinkName;
  m_templatename = pObjTagName;
  m_nInitCt = nInitCt;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_USEROBJECT::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_USEROBJECT* pObjTag = new acpTag_USEROBJECT();

  pObjTag->setOwnerID(acpShort(stream));
  pObjTag->m_name = acpStringIO(stream);
  pObjTag->m_linkname = acpStringIO(stream);
  pObjTag->m_templatename = acpStringIO(stream);
  pObjTag->m_nInitCt = acpInt32(stream);

  return pObjTag;
}


/////////////////////////////////////////////////////////////////////

void acpTag_USEROBJECT::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);
  acpStringIO name(m_name);
  name.writeToStream(stream);
  acpStringIO linkname(m_linkname);
  linkname.writeToStream(stream);
  acpStringIO tname(m_templatename);
  tname.writeToStream(stream);
  acpInt32 initct(m_nInitCt);
  initct.writeToStream(stream);
}
