/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_VISIBLE.cpp 		 		           //
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

#include "acpXmlTag_VISIBLE.h"
#include "acpTag_INSTANCE.h"
#include "acpPkgImport.h"
#include "acpSymPkgImport.h"


/////////////////////////////////////////////////////////////////////

bool VISIBLE::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (((acpSymPkgImport*)m_pImporter)->isGeometry(pTagName)) {
    m_geometry.add((GEOMETRY*)pTag);
    return true;
  } 
    
  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

void VISIBLE::traverse()
{
  m_pImporter->m_nInstanceFlags |= kInstance_Visible;
  acpPkgXML::traverse();
  m_pImporter->m_nInstanceFlags &= ~kInstance_Visible;
}
