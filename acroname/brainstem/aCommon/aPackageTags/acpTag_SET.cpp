/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_SET.cpp		 	 		           //
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

#include "acpTag_SET.h"

#include "acpStringIO.h"
#include "acpShort.h"
#include "acpInt32.h"
#include "acpByte.h"




/////////////////////////////////////////////////////////////////////
// SET = 0x000E
/////////////////////////////////////////////////////////////////////


acpTag_SET::acpTag_SET() :
  acpPackageTag(aTAG_SET, NULL),
  m_nUnit(1),
  m_cParamIndex(0),
  m_cParamSize(0),
  m_pApiCup(NULL),
  m_ulApiCupSize(0)
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_SET::~acpTag_SET()
{
  if (m_pApiCup)
    aMemFree(m_pApiCup);
}


/////////////////////////////////////////////////////////////////////

void acpTag_SET::setData(
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

acpPackageTag* acpTag_SET::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_SET* pSetBlk = new acpTag_SET();

  pSetBlk->setOwnerID(acpShort(stream));
  pSetBlk->m_apifile = acpStringIO(stream);
  pSetBlk->m_nUnit = acpInt32(stream);
  pSetBlk->m_cParamIndex = acpByte(stream);
  pSetBlk->m_cParamSize = acpByte(stream);
  pSetBlk->m_ulApiCupSize = acpInt32(stream);

  unsigned long ulSize = pSetBlk->m_ulApiCupSize;
  if (ulSize) {
    // unspew them bytes
    pSetBlk->m_pApiCup = (char*)aMemAlloc(ulSize);
    for (unsigned int i = 0; i < ulSize; i++) {
      pSetBlk->m_pApiCup[i] = acpByte(stream);
    }
  }

  return pSetBlk;
}


/////////////////////////////////////////////////////////////////////

void acpTag_SET::writeData(
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
