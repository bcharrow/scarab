/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_FLOATVAL.cpp 		                   //
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

#include "acpXmlTag_FLOATVAL.h"
#include "acpString.h"


/////////////////////////////////////////////////////////////////////

void FLOATVAL::addToken(
  const aToken* pToken
)
{
  if (pToken->eType == tkInt) {

    if (m_bNeg)
      m_value = (float)-pToken->v.integer;
    else
      m_value = (float)pToken->v.integer;

    m_bNeg = true; // don't allow trailing neg

  } else if (pToken->eType == tkFloat) {

    if (m_bNeg)
      m_value = -pToken->v.floatVal;
    else  
      m_value = pToken->v.floatVal;

    m_bNeg = true; // don't allow trailing neg

  } else if (!m_bNeg && (pToken->eType == tkSpecial)
	     && (pToken->v.special == '-')) {

    m_bNeg = true; // mark a negative number

  } else {
    acpString msg(tagName());
    msg += " expects a real number";
    tokenError(pToken, msg);
  }
}


/////////////////////////////////////////////////////////////////////

bool FLOATVAL::getDirectFormat(
  acpString& format,
  acpString& example,
  acpString& exampleNotes) 
{
  format = "Real";
  example = "1.65";

  return true; 
}
