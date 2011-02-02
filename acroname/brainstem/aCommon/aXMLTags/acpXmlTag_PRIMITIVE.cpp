/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PRIMITIVE.cpp	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of XML tag class.                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// Copyright 1994-2008. Acroname Inc.                              //
//                                                                 //
// This software is the PRIMITIVE of Acroname Inc.  Any             //
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

#include "acpXmlTag_PRIMITIVE.h"

#include "acpTag_PRIMITIVE.h"
#include "acpXmlTag_apiNAME.h"
#include "acpXmlTag_APIFILE.h"
#include "acpXmlTag_STEMFILE.h"
#include "acpXmlTag_DESCRIPTION.h"
#include "acpXmlTag_LINKNAME.h"
#include "acpPkgImport.h"


/////////////////////////////////////////////////////////////////////

bool PRIMITIVE::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "NAME")) {
    m_name = ((apiNAME*)pTag)->m_name;
  } else if (!aStringCompare(pTagName, "LINKNAME")) {
    m_linkname = ((LINKNAME*)pTag)->m_name;
  } else if (!aStringCompare(pTagName, "DESCRIPTION")) {
    m_description = ((DESCRIPTION*)pTag)->m_description;
  } else if (!aStringCompare(pTagName, "APIFILE")) {
    m_apifile = ((APIFILE*)pTag)->m_filename;
  } else if (!aStringCompare(pTagName, "STEMFILE")) {
    m_stemfile = ((STEMFILE*)pTag)->m_filename;
  } else if (!aStringCompare(pTagName, "PROPERTY")) {
    m_nPropCt++;
    return true;
  } else {
    return false;
  }

  return true;
}


/////////////////////////////////////////////////////////////////////

void PRIMITIVE::traverse()
{
  acpTag_PRIMITIVE* pPrim = new acpTag_PRIMITIVE();
  pPrim->setID((aShort)(m_pImporter->numPackageTags() + 1));

  // build the dynamic tag and add it to the package
  pPrim->setData(m_nOwnerID,
		 m_name,
		 m_linkname,
		 m_apifile,
		 m_stemfile,
		 m_nPropCt);
  m_pImporter->addTag(this, pPrim);

  // set the parent of all the children below this one to this
  aShort saveID = m_pImporter->m_nCurrentOwner;
  m_pImporter->m_nCurrentOwner = pPrim->getID();
  acpPkgXML* pChild;
  acpListIterator<acpPkgXML> children(m_children);
  while ((pChild = children.next())) {
    pChild->setOwnerID(pPrim->getID());
    pChild->traverse();
  }
  m_pImporter->m_nCurrentOwner = saveID;
}
