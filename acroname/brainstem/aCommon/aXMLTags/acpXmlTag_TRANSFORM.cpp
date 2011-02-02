/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_TRANSFORM.cpp 		 		                   //
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

#include "acpXmlTag_TRANSFORM.h"

#include "acpTransform.h"
#include "acpPkgImport.h"
#include "acpSymPkgImport.h"


/////////////////////////////////////////////////////////////////////

TRANSFORM::TRANSFORM(
  acpPkgImport* pImporter,
  acpPkgXML* pParent // = NULL
) :
  GEOMETRY(pImporter, pParent),
  m_pTranslation(NULL)
{
}


/////////////////////////////////////////////////////////////////////

bool TRANSFORM::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "ROTATION")) {
    ROTATION* pRotation = (ROTATION*)pTag;    
    acpVec3 axis(pRotation->m_x,
  	         pRotation->m_y,
  	         pRotation->m_z);
    acpMatrix3 rotation(pRotation->m_a, axis);
    m_rotation = m_rotation * rotation;
  } else if (!aStringCompare(pTagName, "TRANSLATION"))
    return aEXASSIGN(m_pTranslation, pTag);
  
  // we don't know what we are transforming but all let all
  // children through so we can transform them
  return true;
}


/////////////////////////////////////////////////////////////////////

void TRANSFORM::traverse()
{
  acpTransform transform;

  if (m_pTranslation)
    transform.translate(acpVec3(m_pTranslation->m_x,
				m_pTranslation->m_y,
				m_pTranslation->m_z));

  transform.rotate(m_rotation);

  ((acpSymPkgImport*)m_pImporter)->pushTransform(transform);

  acpPkgXML::traverse();

  ((acpSymPkgImport*)m_pImporter)->popTransform();
}
