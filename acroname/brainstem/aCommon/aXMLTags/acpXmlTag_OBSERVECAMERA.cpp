/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_OBSERVECAMERA.cpp 		                   //
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

#include "acpPkgImport.h"
#include "acpXmlTag_OBSERVECAMERA.h"

#include "acpShort.h"
#include "acpVec3.h"
#include "acpPackageTag.h"
#include "acpXmlTag_DISTANCE.h"
#include "acpXmlTag_VIEWS.h"


/////////////////////////////////////////////////////////////////////

bool OBSERVECAMERA::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "DISTANCE")) {
    DISTANCE* pDistance = (DISTANCE*)pTag;
    m_fDistance = *pDistance;
    return true;
  } else if (!aStringCompare(pTagName, "TRANSLATION")) {
    m_pTranslation = (TRANSLATION*)pTag;
    return true;
  } else if (!aStringCompare(pTagName, "TARGET")) {
    m_pTarget = (TARGET*)pTag;
    return true;
  }

  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

void OBSERVECAMERA::traverse()
{
  VIEWS* pViews = (VIEWS*)findAncestor("VIEWS");
  aAssert(pViews);

  acpByte b('O');
  b.writeToStream(pViews->m_buffer);

  acpFloat f(m_fDistance);
  f.writeToStream(pViews->m_buffer);

  // translation
  acpVec3 translation;
  if (m_pTranslation)
    translation = acpVec3(m_pTranslation->m_x,
    			  m_pTranslation->m_y,
    			  m_pTranslation->m_z);
  translation.writeToStream(pViews->m_buffer);

  acpShort target((aShort)0);
  if (m_pTarget) {
    acpPackageTag* pLinked = m_pImporter->getNamedTag(m_pTarget->m_name);
    if (pLinked) {
      target = pLinked->getID();
    } else {
      acpString msg("observe camera references undefined dynamic named ");
      msg += m_pTarget->m_name;
      m_pImporter->error(msg);
    }
  }
  target.writeToStream(pViews->m_buffer);
}
