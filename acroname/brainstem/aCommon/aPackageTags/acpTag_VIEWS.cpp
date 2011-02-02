/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_VIEWS.cpp	 	 		           //
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

#include "acpTag_VIEWS.h"

#include "acpException.h"
#include "acpShort.h"


/////////////////////////////////////////////////////////////////////
// VIEWS = 0x0001
/////////////////////////////////////////////////////////////////////

acpTag_VIEWS::~acpTag_VIEWS()
{
  if (m_pData)
    aMemFree(m_pData);
}


/////////////////////////////////////////////////////////////////////

void acpTag_VIEWS::setData(
  const aShort nSize,
  const aByte* pData
)
{
  m_nDataSize = nSize;
  
  // create some temporary space to read in the data
  m_pData = (aByte*)aMemAlloc(nSize * sizeof(aByte));
  if (!m_pData)
    throw acpException(aErrMemory, "allocating view data");
  
  aMemCopy(m_pData, pData, nSize);
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_VIEWS::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_VIEWS* pViews = new acpTag_VIEWS();

  pViews->m_nDataSize = acpShort(stream);
  
  // create some temporary space to read in the data
  pViews->m_pData = 
  	(aByte*)aMemAlloc(pViews->m_nDataSize * sizeof(aByte));
  if (!pViews->m_pData)
    throw acpException(aErrMemory, "allocating view data");

  aErr err;
  if (aStream_Read(aStreamLibRef(stream), stream, 
  		   pViews->m_pData, pViews->m_nDataSize, &err))
    throw acpException(err, "reading data for for views");
  
  return pViews;  
}


/////////////////////////////////////////////////////////////////////

void acpTag_VIEWS::writeData(
  aStreamRef stream
) const
{
  acpShort size(m_nDataSize);
  size.writeToStream(stream);

  aErr err;
  if (aStream_Write(aStreamLibRef(stream), stream, 
 	 	    m_pData, m_nDataSize, &err))
    throw acpException(err, "writing data for for views");
}
