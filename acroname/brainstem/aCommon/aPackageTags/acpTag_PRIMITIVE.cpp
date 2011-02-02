/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_PRIMITIVE.cpp	 	 		           //
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

#include "acpTag_PRIMITIVE.h"

#include "acpStringIO.h"
#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// PRIMITIVE = 0x0010
/////////////////////////////////////////////////////////////////////


acpTag_PRIMITIVE::acpTag_PRIMITIVE() :
  acpPackageTag(aTAG_PRIMITIVE, NULL),
  m_pApiCup(NULL),
  m_ulApiCupSize(0),
  m_pStemCup(NULL),
  m_ulStemCupSize(0)
{
}


/////////////////////////////////////////////////////////////////////

acpTag_PRIMITIVE::~acpTag_PRIMITIVE()
{ 
  if (m_pApiCup)
    aMemFree(m_pApiCup);
  if (m_pStemCup)
    aMemFree(m_pStemCup);
}


/////////////////////////////////////////////////////////////////////

void acpTag_PRIMITIVE::setData(
  const aShort nOwnerID,
  const char* pName,
  const char* pLinkName,
  const char* pApiName,
  const char* pStemName,
  const aShort nPropCt
)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  m_linkname = pLinkName;
  m_apifile = pApiName;
  m_stemfile = pStemName;
  m_nPropCt = nPropCt;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_PRIMITIVE::factoryFromStream(
  aStreamRef stream
)
{
  unsigned long ulSize;
  acpTag_PRIMITIVE* pPrim = new acpTag_PRIMITIVE();

  pPrim->setOwnerID(acpShort(stream));
  pPrim->m_name = acpStringIO(stream);
  pPrim->m_linkname = acpStringIO(stream);

  pPrim->m_apifile = acpStringIO(stream);
  ulSize = acpInt32(stream);
  pPrim->m_ulApiCupSize = ulSize;
  if (ulSize) {
    // unspew them bytes
    pPrim->m_pApiCup = (char*)aMemAlloc(ulSize);
    for (unsigned int i = 0; i < ulSize; i++) {
      pPrim->m_pApiCup[i] = acpByte(stream);
    }
  }

  pPrim->m_stemfile = acpStringIO(stream);
  ulSize = acpInt32(stream);
  pPrim->m_ulStemCupSize = ulSize;
  if (ulSize) {
    // unspew them bytes
    pPrim->m_pStemCup = (char*)aMemAlloc(ulSize);
    for (unsigned int i = 0; i < ulSize; i++) {
      pPrim->m_pStemCup[i] = acpByte(stream);
    }
  }
  
  pPrim->m_nPropCt = acpShort(stream);

  return pPrim;
}


/////////////////////////////////////////////////////////////////////

void acpTag_PRIMITIVE::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  acpStringIO name(m_name);
  name.writeToStream(stream);
  acpStringIO linkname(m_linkname);
  linkname.writeToStream(stream);

  acpStringIO sapi(m_apifile);
  sapi.writeToStream(stream);

  acpInt32 ulApiCupSize(m_ulApiCupSize);
  ulApiCupSize.writeToStream(stream);
  
  if (m_ulApiCupSize) {
    // spew them bytes
    for (unsigned int i = 0; i < m_ulApiCupSize; i++) {
      acpByte b(m_pApiCup[i]);
      b.writeToStream(stream);
    }
  }

  acpStringIO sstem(m_stemfile);
  sstem.writeToStream(stream);

  acpInt32 ulStemCupSize(m_ulStemCupSize);
  ulStemCupSize.writeToStream(stream);
  
  if (m_ulStemCupSize) {
    // spew them bytes
    for (unsigned int i = 0; i < m_ulStemCupSize; i++) {
      acpByte b(m_pStemCup[i]);
      b.writeToStream(stream);
    }
  }
  
  acpShort nProps(m_nPropCt);
  nProps.writeToStream(stream);
}
