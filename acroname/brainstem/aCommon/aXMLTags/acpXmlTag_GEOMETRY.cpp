/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_GEOMETRY.cpp 		                   //
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

#include "acpShort.h"
#include "acpXmlTag_GEOMETRY.h"
#include "acpXmlTag_NAME.h"
#include "acpPkgImport.h"
#include "acpSymPkgImport.h"
#include "acpTag_INSTANCE.h"


/////////////////////////////////////////////////////////////////////

bool GEOMETRY::equalGeometry(
  const acpPkgXML* pTag) const
{
  const char* pN1 = tagName();
  const char* pN2 = pTag->tagName();
  return !(aStringCompare(pN1, pN2)); 
}


/////////////////////////////////////////////////////////////////////

bool GEOMETRY::addTag(
  acpPkgXML* pTag)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "NAME"))
    return aEXASSIGN(m_pName, pTag);
  else if (!((acpSymPkgImport*)m_pImporter)->isGeometry(pTagName))
    return false;

  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////
    
void GEOMETRY::addPackageTag(
  acpPackageTag* pTag)
{
  m_pImporter->addTag(this, pTag);
  
  aInt32 id = m_pImporter->numPackageTags();
  m_tagIndices.add(new acpShort((aShort)id));
}


/////////////////////////////////////////////////////////////////////
// called by the DYNAMIC which has GEOMETRY children

void GEOMETRY::traverse()
{
//  addGeometry();

  // build up a list of referenced geometries
  aShort save = m_pImporter->m_nCurrentOwner;
  m_pImporter->m_nCurrentOwner = 0;
  acpList<acpShort> geometry;
  const acpList<acpShort>* pGeometries = 
    ((acpSymPkgImport*)m_pImporter)->ensureGeometry(this);
  aLISTITERATE(acpShort, *pGeometries, pID) {
    geometry.add(new acpShort(*pID));
  }
  m_pImporter->m_nCurrentOwner = save;

  // this is the instance for the dynamic object
  acpTag_INSTANCE* pInstance = new acpTag_INSTANCE();
  pInstance->setID((aShort)(m_pImporter->numPackageTags() + 1));

  // start with no translation or rotation
//  acpVec3 translation;
//  if (m_pTranslation)
//    translation = acpVec3(m_pTranslation->m_x,
//    		          m_pTranslation->m_y,
//    		          m_pTranslation->m_z);

//  acpMatrix3 rotation;
//  if (m_pRotation)
//    rotation = acpMatrix3((double)-m_pRotation->m_a, 
//    			  acpVec3(m_pRotation->m_x,
//    				  m_pRotation->m_y,
//    				  m_pRotation->m_z));

  acpTransform t = ((acpSymPkgImport*)m_pImporter)->currentTransform();
//  t.translate(translation);
//  t.rotate(rotation);
  
  pInstance->setData(m_pImporter->m_nCurrentOwner,
  		     "",
  		     geometry,
  		     m_pImporter->m_nInstanceFlags,
  		     t);

  m_pImporter->addTag(this, pInstance);
}
