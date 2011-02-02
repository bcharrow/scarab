/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_SET.cpp	 		                   //
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

#include "acpXmlTag_SET.h"

#include "acpTag_SET.h"
#include "acpXmlTag_TYPE.h"
#include "acpXmlTag_APIFILE.h"
#include "acpXmlTag_CONVFAC.h"
#include "acpXmlTag_PARAMSIZE.h"
#include "acpXmlTag_INDEX.h"

#include "acpXmlTag_TITLE.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

bool SET::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "APIFILE")) {
    m_apifile = ((APIFILE*)pTag)->m_filename;
  } else if (!aStringCompare(pTagName, "CONVFAC")) {
    m_nUnit = ((CONVFAC*)pTag)->m_nConvFac;
  } else if (!aStringCompare(pTagName, "PARAMSIZE")) {
    m_cParamSize = (aByte)(((PARAMSIZE*)pTag)->m_nSize);
  } else if (!aStringCompare(pTagName, "INDEX")) {
    m_cParamIndex = (aByte)(((INDEX*)pTag)->m_nIndex);
  } else {
    return false;
  }

  return true;
}


/////////////////////////////////////////////////////////////////////

void SET::traverse()
{
  acpTag_SET* pSetTag = new acpTag_SET();

  pSetTag->setData(
    m_pImporter->m_nCurrentOwner,
    m_apifile,
    m_nUnit,
    m_cParamIndex,
    m_cParamSize);

  m_pImporter->addTag(this, pSetTag);
}
