/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_SPHERE.cpp 		 		           //
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

#include "acpXmlTag_SPHERE.h"

#include "acpTag_SPHERE.h"
#include "acpVec3.h"
#include "acpPkgImport.h"


/////////////////////////////////////////////////////////////////////

bool SPHERE::equalGeometry(
  const acpPkgXML* pTag
) const
{
  if (!GEOMETRY::equalGeometry(pTag))
    return false;

  SPHERE* pOther = (SPHERE*)pTag;

  if (m_pColor || pOther->m_pColor) {
    if (!m_pColor || !pOther->m_pColor)
      return false;
    if (!(*m_pColor == *(pOther->m_pColor)))
      return false;
  }

  return (m_fRadius == pOther->m_fRadius);
}


/////////////////////////////////////////////////////////////////////

void SPHERE::addGeometry()
{
  acpTag_SPHERE* pSphereTag = new acpTag_SPHERE();

  acpVec3 color(0.5f, 0.5f, 0.5f);
  if (m_pColor)
    color = acpVec3(m_pColor->m_r, 
    		    m_pColor->m_g, 
    		    m_pColor->m_b);

  pSphereTag->setData(m_pImporter->m_nCurrentOwner,
		      color,
    		      m_fRadius);

  addPackageTag(pSphereTag);
}


/////////////////////////////////////////////////////////////////////

void SPHERE::addToken(
  const aToken* pToken
)
{
  if (m_fRadius != 0) {
    tokenError(pToken, "SPHERE requires only one direct value");
  } else if (pToken->eType == tkFloat) {
    m_fRadius = pToken->v.floatVal;
  } else if (pToken->eType == tkInt) {
    m_fRadius = pToken->v.integer;
  } else {
    tokenError(pToken, "SPHERE expects a numeric radius");
  }
}


/////////////////////////////////////////////////////////////////////

bool SPHERE::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "COLOR")) {
    m_pColor = (COLOR*)pTag;
  } else {
    return false;
  }

  return true;
}


