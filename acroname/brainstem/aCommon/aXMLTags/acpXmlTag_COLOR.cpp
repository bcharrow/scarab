/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_COLOR.cpp 		 		           //
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

#include "acpXmlTag_GEOMETRY.h"
#include "acpXmlTag_COLOR.h"


/////////////////////////////////////////////////////////////////////

const bool COLOR::operator==(
  const COLOR& rhs) const
{
  return ((m_r == rhs.m_r)
  	  && (m_g == rhs.m_g)
  	  && (m_b == rhs.m_b));
}


/////////////////////////////////////////////////////////////////////

void COLOR::addToken(
  const aToken* pToken
)
{
  float val = 0;
  if (pToken->eType == tkFloat) {
    val = pToken->v.floatVal;
  } else if (pToken->eType == tkInt) {
    val = pToken->v.integer;
  } else {
    tokenError(pToken, "COLOR expects numbers");
  }
  switch (m_count++) {
  case 0: m_r = val; break;
  case 1: m_g = val; break;
  case 2: m_b = val; break;
  default:
    tokenError(pToken, "COLOR expects only 3 numbers");
    break;
  } // switch
}
