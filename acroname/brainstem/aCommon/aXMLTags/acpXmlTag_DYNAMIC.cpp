/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_DYNAMIC.cpp 	 		                   //
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

#include "acpXmlTag_DYNAMIC.h"
#include "acpTag_DYNAMIC.h"
#include "acpSymPkgImport.h"



/////////////////////////////////////////////////////////////////////

bool DYNAMIC::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "NAME"))
    return aEXASSIGN(m_pName, pTag);
  else if (!aStringCompare(pTagName, "TRANSLATION"))
    return aEXASSIGN(m_pTranslation, pTag);
  else if (!aStringCompare(pTagName, "ROTATION")) 
    return aEXASSIGN(m_pRotation, pTag);
  else if (!aStringCompare(pTagName, "MASS"))
    return aEXASSIGN(m_pMass, pTag);
  else if (!aStringCompare(pTagName, "FRICTION"))
    return aEXASSIGN(m_pFriction, pTag);
  else if (!aStringCompare(pTagName, "ERP"))
    return aEXASSIGN(m_pSoftERP, pTag);
  else if (!aStringCompare(pTagName, "CFM"))
    return aEXASSIGN(m_pSoftCFM, pTag);
  else if (!aStringCompare(pTagName, "BOUNCE"))
    return aEXASSIGN(m_pBounce, pTag);
  else if (!aStringCompare(pTagName, "VISIBLE"))
    return aEXASSIGN(m_pVisible, pTag);
  else if (!aStringCompare(pTagName, "DYNAMIC")) {
    m_children.add((DYNAMIC*)pTag);
    return true;
  } else if (!aStringCompare(pTagName, "MOTOR")) {
    return true;
  } else if (!aStringCompare(pTagName, "SERVO")) {
    return true;
  } else if (!aStringCompare(pTagName, "PLUGIN")) {
    return true;
  } else if (((acpSymPkgImport*)m_pImporter)->isGeometry(pTagName)) {
    // children are just held as children of the node
    return true;
  }

  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

void DYNAMIC::traverse()
{
  acpTag_DYNAMIC* pDynamicTag = new acpTag_DYNAMIC();
  pDynamicTag->setID((aShort)(m_pImporter->numPackageTags() + 1));

  acpString name;
  if (m_pName)
    name = m_pName->m_name;


  aFloat fMass = 0.5f;
  if (m_pMass)
    fMass = (float)*m_pMass;
  else {
    acpString msg("DYNAMIC");
    if (m_pName) {
      msg += " named ";
      msg += m_pName->m_name;
    }
    msg += " requires a mass value";
    tagError(msg);
  }

  aFloat fFriction = 0.0f;
  if (m_pFriction)
    fFriction = *m_pFriction;
  
  aFloat fSoftERP = 1.0f;
  if (m_pSoftERP)
    fSoftERP = *m_pSoftERP;

  aFloat fSoftCFM = 0.1f;
  if (m_pSoftCFM)
    fSoftCFM = *m_pSoftCFM;
  
  aFloat fBounce = 0.1f;
  if (m_pBounce)
    fBounce = *m_pBounce;
  
  acpVec3 position;
  if (m_pTranslation)
    position = acpVec3(m_pTranslation->m_x,
    		       m_pTranslation->m_y,
    		       m_pTranslation->m_z);

  acpMatrix3 rotation;
  if (m_pRotation)
    rotation = acpMatrix3((double)-m_pRotation->m_a, 
    			  acpVec3(m_pRotation->m_x,
    				  m_pRotation->m_y,
    				  m_pRotation->m_z));

  // build the dynamic tag and add it to the package
  pDynamicTag->setData(m_nOwnerID,
  		       name,
		       fMass,
		       fFriction,
		       fSoftERP,
		       fSoftCFM,
		       fBounce,
    		       position,
    		       rotation);
  m_pImporter->addTag(this, pDynamicTag);

  // set the parent of all the children below this one to this
  aShort saveID = m_pImporter->m_nCurrentOwner;
  m_pImporter->m_nCurrentOwner = pDynamicTag->getID();
  acpPkgXML* pChild;
  acpListIterator<acpPkgXML> children(m_children);
  while ((pChild = children.next())) {
    pChild->setOwnerID(pDynamicTag->getID());
    pChild->traverse();
  }
  m_pImporter->m_nCurrentOwner = saveID;
}
