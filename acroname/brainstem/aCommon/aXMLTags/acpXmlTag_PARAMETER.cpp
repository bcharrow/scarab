/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PARAMETER.cpp 		 		                   //
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

#include "acpXmlTag_PARAMETER.h"



/////////////////////////////////////////////////////////////////////

PARAMETER::~PARAMETER()
{
  if (m_pValue) {
    delete m_pValue;
    m_pValue = NULL;  	
  }
}


/////////////////////////////////////////////////////////////////////

void PARAMETER::addToken(
  const aToken* pToken
)
{
  if (m_pValue) {
    tokenError(pToken, "PARAMETER expects a single token");
  } else {
    switch (pToken->eType) {
    	
    case tkInt:
      m_pValue = new acpValue(pToken->v.integer);
      break;

    case tkFloat:
      m_pValue = new acpValue(pToken->v.floatVal);
      break;

    case tkString:
      m_pValue = new acpValue(pToken->v.string);
      break;

    case tkIdentifier:
      tokenError(pToken, "PARAMETER does not accept an identifier");      
      break;

    default:
      tokenError(pToken, "PARAMETER has invalid data");
    }	
  }
}


/////////////////////////////////////////////////////////////////////

bool PARAMETER::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "NAME")) {  	
    m_pName = (NAME*)pTag;
    return true;
  }

  return acpPkgXML::addTag(pTag);
}
