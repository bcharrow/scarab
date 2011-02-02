/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_GET.cpp		 	 		           //
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

#include "acpTag_GET.h"

#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// GET = 0x000F
/////////////////////////////////////////////////////////////////////


acpTag_GET::acpTag_GET() :
  acpPackageTag(aTAG_GET, NULL),
  m_nUnit(1),
  m_cParamIndex(0),
  m_cParamSize(0),
  m_pApiCup(NULL),
  m_ulApiCupSize(0)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_GET::~acpTag_GET()
{
  if (m_pApiCup)
    aMemFree(m_pApiCup);
}


/////////////////////////////////////////////////////////////////////

void acpTag_GET::setData(
  const aShort nOwnerID,
  const char* pApiFile,
  const aInt32 nUnit,
  const aByte cParamIndex,
  const aByte cParamSize
)
{
  setOwnerID(nOwnerID);
  m_apifile = pApiFile;
  m_nUnit = nUnit;
  m_cParamIndex = cParamIndex;
  m_cParamSize = cParamSize;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_GET::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_GET* pGetBlk = new acpTag_GET();

  pGetBlk->setOwnerID(acpShort(stream));
  pGetBlk->m_apifile = acpStringIO(stream);
  pGetBlk->m_nUnit = acpInt32(stream);
  pGetBlk->m_cParamIndex = acpByte(stream);
  pGetBlk->m_cParamSize = acpByte(stream);
  pGetBlk->m_ulApiCupSize = acpInt32(stream);

  unsigned long ulSize = pGetBlk->m_ulApiCupSize;
  if (ulSize) {
    // unspew them bytes
    pGetBlk->m_pApiCup = (char*)aMemAlloc(ulSize);
    for (unsigned int i = 0; i < ulSize; i++) {
      pGetBlk->m_pApiCup[i] = acpByte(stream);
    }
  }

  return pGetBlk;
}


/////////////////////////////////////////////////////////////////////

void acpTag_GET::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  acpStringIO apifile(m_apifile);
  apifile.writeToStream(stream);

  acpInt32 nUnit(m_nUnit);
  nUnit.writeToStream(stream);

  acpByte cParamIndex(m_cParamIndex);
  cParamIndex.writeToStream(stream);
  
  acpByte cParamSize(m_cParamSize);
  cParamSize.writeToStream(stream);

  acpInt32 ulApiCupSize(m_ulApiCupSize);
  ulApiCupSize.writeToStream(stream);
  
  if (m_ulApiCupSize) {
    // spew them bytes
    for (unsigned int i = 0; i < m_ulApiCupSize; i++) {
      acpByte b(m_pApiCup[i]);
      b.writeToStream(stream);
    }
  }
}
