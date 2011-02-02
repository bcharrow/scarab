/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_TEMPLATE.cpp	 	 		           //
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

#include "acpTag_TEMPLATE.h"

#include "acpStringIO.h"
#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// TEMPLATE = 0x0011
/////////////////////////////////////////////////////////////////////


acpTag_TEMPLATE::acpTag_TEMPLATE() :
  acpPackageTag(aTAG_TEMPLATE, NULL)
{
}


/////////////////////////////////////////////////////////////////////

acpTag_TEMPLATE::~acpTag_TEMPLATE()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_TEMPLATE::setData(
  const aShort nOwnerID,
  const char* pName,
  const aShort nPropCt,
  const aInt32 nBufferSize
)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  m_nPropCt = nPropCt;
  m_nBufferSize = nBufferSize;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_TEMPLATE::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_TEMPLATE* pTemplate = new acpTag_TEMPLATE();

  pTemplate->setOwnerID(acpShort(stream));
  pTemplate->m_name = acpStringIO(stream);
  pTemplate->m_nPropCt = acpShort(stream);
  pTemplate->m_nBufferSize = acpInt32(stream);

  return pTemplate;
}


/////////////////////////////////////////////////////////////////////

void acpTag_TEMPLATE::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);
  acpStringIO name(m_name);
  name.writeToStream(stream);
  acpShort nProps(m_nPropCt);
  nProps.writeToStream(stream);
  acpInt32 nBufferSize(m_nBufferSize);
  nBufferSize.writeToStream(stream);
}
