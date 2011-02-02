/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_STATIC.cpp 		 		           //
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
#include "acpXmlTag_STATIC.h"
#include "acpTag_INSTANCE.h"
#include "acpSymPkgImport.h"


/////////////////////////////////////////////////////////////////////

bool STATIC::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (((acpSymPkgImport*)m_pImporter)->isGeometry(pTagName)) {
    m_geometry.add((GEOMETRY*)pTag);
    return true;
  } else if (!aStringCompare(pTagName, "TRANSLATION")) {
    m_pTranslation = ((TRANSLATION*)pTag);
    return true;
  } else if (!aStringCompare(pTagName, "ROTATION")) {
    m_pRotation = ((ROTATION*)pTag);
    return true;
  } else {
    return acpPkgXML::addTag(pTag);
  }
}


/////////////////////////////////////////////////////////////////////
// This should walk the list of geometries and make sure they
// are added to the global list of geometries.  It then creates
// and instance for these geometry elements.

void STATIC::traverse()
{
  // build a list of the geometry ID's this instance needs
  // from the global list of geometries
  acpList<acpShort> geometry;
  aLISTITERATE(GEOMETRY, m_geometry, pGEOMETRY) {
    const acpList<acpShort>* pGeometries = 
      ((acpSymPkgImport*)m_pImporter)->ensureGeometry(pGEOMETRY);
    acpShort* pID;
    acpListIterator<acpShort> geomIterator(*pGeometries);
    while ((pID = geomIterator.next()))
      geometry.add(new acpShort(*pID));
  }

  // this is the instance for the static
  acpTag_INSTANCE* pInstance = new acpTag_INSTANCE();
  pInstance->setID((aShort)(m_pImporter->numPackageTags() + 1));

  // start with no translation or rotation
  acpVec3 translation;
  if (m_pTranslation)
    translation = acpVec3(m_pTranslation->m_x,
    		          m_pTranslation->m_y,
    		          m_pTranslation->m_z);
    		          
  acpMatrix3 rotation;
  if (m_pRotation)
    rotation = acpMatrix3((double)-m_pRotation->m_a, 
    			  acpVec3(m_pRotation->m_x,
    				  m_pRotation->m_y,
    				  m_pRotation->m_z));

  // now, add the current world transform to the static's
  // values
  acpTransform t = ((acpSymPkgImport*)m_pImporter)->currentTransform();
  t.rotate(rotation);
  t.translate(translation);

  pInstance->setData(m_pImporter->m_nCurrentOwner,
  		     "",
  		     geometry,
  		     (char)(m_pImporter->m_nInstanceFlags 
  		            | kInstance_Static),
  		     t);

  m_pImporter->addTag(this, pInstance);  
}
