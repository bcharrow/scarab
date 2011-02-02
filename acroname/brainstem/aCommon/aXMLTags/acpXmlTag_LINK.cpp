/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_LINK.cpp  			                   //
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

#include "acpXmlTag_LINK.h"
#include "acpXmlTag_apiNAME.h"
#include "acpTag_LINK.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

bool LINK::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "NAME")) {
    m_linkname = ((apiNAME*)pTag)->m_name;
//  } else if (!aStringCompare(pTagName, "DEFAULT")) {
//    m_svalue = ((DEFAULT*)pTag)->m_data;
  } else {
    return false;
  }

  return true;
}

/*
void LINK::addToken(
  const aToken* pToken
)
{
  if (pToken->eType == tkString) {
    m_linkname += pToken->v.string;
  } else {
    tokenError(pToken, "LINK expects a string");
  }
}
*/


/////////////////////////////////////////////////////////////////////

void LINK::traverse()
{
  acpTag_LINK* pLinkTag = new acpTag_LINK();
  pLinkTag->setID((aShort)(m_pImporter->numPackageTags() + 1));

  // build the dynamic tag and add it to the package
  pLinkTag->setData(m_nOwnerID,
		 m_linkname,
		 "");
  m_pImporter->addTag(this, pLinkTag);
}
