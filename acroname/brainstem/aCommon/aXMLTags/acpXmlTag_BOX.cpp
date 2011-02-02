/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_BOX.cpp	 		                   //
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

#include "acpXmlTag_BOX.h"

#include "acpTag_BOX.h"
#include "acpVec3.h"
#include "acpPkgImport.h"


/////////////////////////////////////////////////////////////////////

bool BOX::equalGeometry(
  const acpPkgXML* pTag
) const
{
  if (!GEOMETRY::equalGeometry(pTag))
    return false;

  BOX* pOther = (BOX*)pTag;

  if (!m_pDimensions 
       || !pOther->m_pDimensions 
       || (m_pDimensions->m_x != pOther->m_pDimensions->m_x)
       || (m_pDimensions->m_y != pOther->m_pDimensions->m_y)
       || (m_pDimensions->m_z != pOther->m_pDimensions->m_z))
    return false;

  if (m_pColor
      && ((m_pColor->m_r != pOther->m_pColor->m_r)
          || (m_pColor->m_g != pOther->m_pColor->m_g)
          || (m_pColor->m_b != pOther->m_pColor->m_b)))
    return false;

  return true;
}


/////////////////////////////////////////////////////////////////////

bool BOX::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "COLOR"))
    return aEXASSIGN(m_pColor, pTag);
  else if (!aStringCompare(pTagName, "DIMENSIONS"))
    return aEXASSIGN(m_pDimensions, pTag);

  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

void BOX::addGeometry()
{
  acpTag_BOX* pBoxTag = new acpTag_BOX();

  acpVec3 color(0.5f, 0.5f, 0.5f);
  if (m_pColor)
    color = acpVec3(m_pColor->m_r, 
    		    m_pColor->m_g, 
    		    m_pColor->m_b);

  acpVec3 dimensions(1.0f, 1.0f, 1.0f);
  if (m_pDimensions)
    dimensions = acpVec3(m_pDimensions->m_x, 
    		         m_pDimensions->m_y, 
    		         m_pDimensions->m_z);

  pBoxTag->setData(m_pImporter->m_nCurrentOwner,
		   color,
		   dimensions);

  addPackageTag(pBoxTag);
}
