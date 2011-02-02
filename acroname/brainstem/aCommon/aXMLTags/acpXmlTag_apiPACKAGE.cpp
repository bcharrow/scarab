/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_apiPACKAGE.cpp 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of XML tag class.                   //
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

#include "acpXmlTag_apiPACKAGE.h"
#include "acpXmlTag_DISTANCEUNIT.h"
#include "acpXmlTag_ANGLEUNIT.h"
#include "acpXmlTag_MASSUNIT.h"
#include "acpXmlTag_DESCRIPTION.h"
#include "acpTag_apiPACKAGE.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

bool apiPACKAGE::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (
      !aStringCompare(pTagName, "PROPERTY")
   || !aStringCompare(pTagName, "PRIMITIVE")
   || !aStringCompare(pTagName, "TEMPLATE")
   || !aStringCompare(pTagName, "USEROBJECT")
   || !aStringCompare(pTagName, "LINK")
  )
    return true;

  if (!aStringCompare(pTagName, "DISTANCEUNIT")) {
    m_unitDistance = ((DISTANCEUNIT*)pTag)->m_cUnitCode;
  } else if (!aStringCompare(pTagName, "DESCRIPTION")) {
    m_description = ((DESCRIPTION*)pTag)->m_description;
  } else if (!aStringCompare(pTagName, "ANGLEUNIT")) {
    m_unitAngle = ((ANGLEUNIT*)pTag)->m_cUnitCode;
  } else if (!aStringCompare(pTagName, "MASSUNIT")) {
    m_unitMass = ((MASSUNIT*)pTag)->m_cUnitCode;
  } else {

    return false;
  }
  
  return true;
}



/////////////////////////////////////////////////////////////////////

void apiPACKAGE::end()
{
  acpTag_apiPACKAGE* pPkgData = new acpTag_apiPACKAGE();

  pPkgData->setData(
    m_pImporter->m_nCurrentOwner,
    m_unitDistance,
    m_unitAngle,
    m_unitMass);

  m_pImporter->addTag(this, pPkgData);

  traverse();
}
