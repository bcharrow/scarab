/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_DEFAULT.cpp 	 		                   //
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

#include "acpXmlTag_DEFAULT.h"
#include "aUtil.h"


/////////////////////////////////////////////////////////////////////

void DEFAULT::addToken(
  const aToken* pToken
)
{
  char buff[aMAXIDENTIFIERLEN];
  char cuff[aMAXIDENTIFIERLEN];
  aStringCopy(buff, "");
  aStringCopy(cuff, "");

  // string is the most portable format for storing
  // the default value in the output package
  if (pToken->eType == tkInt) {
    int n = pToken->v.integer;
    if (m_bneg) {
      n = -n;
    }
    aStringFromInt(buff, n);
    m_data += buff;
  } else if (pToken->eType == tkFloat) {

    /*
    int k;
    int fracbits = 8;
    float fac = 1.0f;
    for (k = 0; k < fracbits; k++) fac *= 10.f;

    float f = pToken->v.floatVal;
    int nwhole = f;
    int rfrac = ((f - (float)nwhole) * fac);

    if (m_bneg) {
      aStringCat(buff, "-");
    }
    aStringFromInt(cuff, nwhole);
    aStringCat(buff, cuff);
    aStringCat(buff, ".");

    for (k = 0; k < fracbits; k++) {
      if (rfrac < fac)
        aStringCat(buff, "0");
      fac /= 10.f;
    }

    aStringFromInt(cuff, rfrac);
    aStringCat(buff, cuff);
    */

    float f = pToken->v.floatVal;
    if (m_bneg) {
      f = -f;
    }
    aString_FormatFloat(f, buff);
    m_data += buff;

  } else if (pToken->eType == tkString) {
    m_data += pToken->v.string;
  } else if (pToken->eType == tkSpecial) {
    if (pToken->v.special == '-') {
      m_bneg = true;
    }
  } else {
    tokenError(pToken, "bad DEFAULT value");
  }
}
