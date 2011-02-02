/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PLANE.cpp 	 		                   //
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

#include "acpXmlTag_PLANE.h"

#include "acpXmlTag_NORMAL.h"
#include "acpTag_PLANE.h"
#include "acpVec3.h"
#include "acpPkgImport.h"


/////////////////////////////////////////////////////////////////////

bool PLANE::equalGeometry(
  const acpPkgXML* pTag
) const
{
  if (!GEOMETRY::equalGeometry(pTag))
    return false;
  
  PLANE* pOther = (PLANE*)pTag;
  return ((m_pNormal->m_x == pOther->m_pNormal->m_x)
          && (m_pNormal->m_y == pOther->m_pNormal->m_y)
          && (m_pNormal->m_z == pOther->m_pNormal->m_z)
	  && (m_fHeight == pOther->m_fHeight));
}


/////////////////////////////////////////////////////////////////////

bool PLANE::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "COLOR")) {
    m_pColor = (COLOR*)pTag;
  } else if (!aStringCompare(pTagName, "NORMAL")) {
    m_pNormal = (NORMAL*)pTag;
  } else {
    return false;
  }

  return true;
}


/////////////////////////////////////////////////////////////////////

void PLANE::addGeometry()
{
  acpTag_PLANE* pPlaneTag = new acpTag_PLANE();

  acpVec3 color(0.5f, 0.5f, 0.5f);
  if (m_pColor)
    color = acpVec3(m_pColor->m_r, 
    		    m_pColor->m_g, 
    		    m_pColor->m_b);

  acpVec3 normal(0.0f, 0.0f, 1.0f);
  if (m_pNormal)
    normal = acpVec3(m_pNormal->m_x, 
    		     m_pNormal->m_y, 
    		     m_pNormal->m_z);
    		     
  float fHeight = 0.0f;

  pPlaneTag->setData(m_pImporter->m_nCurrentOwner,
		     color,
		     normal,
    		     fHeight);
  
  addPackageTag(pPlaneTag);
}
