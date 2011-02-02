/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPkgXML.cpp				                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: XML Package Tag class.				   //
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

#include "acpPkgXML.h"

#include "acpPkgImport.h"


/////////////////////////////////////////////////////////////////////

acpPkgXML::acpPkgXML(
  acpPkgImport* pImporter,
  acpPkgXML* pParent // =  NULL
) :
  m_pImporter(pImporter),
  m_pParent(pParent),
  m_nOwnerID(0)
{
  m_ioRef = pImporter->m_ioRef;
}


/////////////////////////////////////////////////////////////////////

acpPkgXML::~acpPkgXML()
{
}


/////////////////////////////////////////////////////////////////////

void acpPkgXML::addToken(
  const aToken* pToken
)
{
  acpString msg = tagName();
  msg += " has no direct data";
  tokenError(pToken, msg);
}


/////////////////////////////////////////////////////////////////////

bool acpPkgXML::addTag(
  acpPkgXML* pTag
)
{  
  return false;
}


/////////////////////////////////////////////////////////////////////

void acpPkgXML::traverse()
{
  acpPkgXML* pChild;
  acpListIterator<acpPkgXML> children(m_children);
  while ((pChild = children.next()))
    pChild->traverse();
}


/////////////////////////////////////////////////////////////////////

acpPkgXML* acpPkgXML::findAncestor(
  const char* pTagName
) const
{
  // look up the tree to find the view parent
  acpPkgXML* pTemp = m_pParent;
  while (pTemp && aStringCompare(pTemp->tagName(), pTagName))
    pTemp = pTemp->parent();
  
  return pTemp;
}


/////////////////////////////////////////////////////////////////////

void acpPkgXML::tokenError(
  const aToken* pToken,
  const char* pMsg
)
{
  aTokenInfo ti;
  aErr err;

  if (pToken) {
    if (aToken_GetInfo(m_ioRef, pToken, &ti, &err))
      throw acpException(err, "getting token data");
  } else {
    ti.nLine = 0;
    ti.nColumn = 0;
  }

  const char* data[2];
  data[0] = "unknown";
  data[1] = pMsg;
  m_pImporter->xmlErr(tagTokenError, 
  	    	      ti.nLine, 
  	    	      ti.nColumn, 
  	    	      2, 
 	 	      data, 
	  	      m_pImporter);
}


/////////////////////////////////////////////////////////////////////

void acpPkgXML::tagError(
  const char* pMsg
)
{
  aTokenInfo ti;
 // aErr err;
 // if (aToken_GetInfo(m_ioRef, m_pStartToken, &ti, &err))
 //   throw acpException(err, "getting token data");
  ti.nLine = 0;
  ti.nColumn = 0;

  const char* data[2];
  data[0] = "unknown";
  data[1] = pMsg;
  m_pImporter->xmlErr(tagTokenError, 
  	    	      ti.nLine, 
  	    	      ti.nColumn, 
  	    	      2, 
 	 	      data, 
	  	      m_pImporter);
}


/////////////////////////////////////////////////////////////////////
// This is assigned from the aEXASSIGN macro which looks like:
//
//   #define aEXASSIGN(p, t) assignExclusive(&p, t)
//
// It should check to make sure the pointer is not already assigned
// and then assign if it is.

bool acpPkgXML::assignExclusive(
  void* vp, //(acpPkgXML**)
  acpPkgXML* pTag) 
{
  // make sure it is exclusive
  acpPkgXML** pp = (acpPkgXML**)vp;
  if (*pp) {
    acpString msg(tagName());
    msg += " only accepts one ";
    msg += pTag->tagName();
    tokenError(NULL, msg);
    return false;
  }

  // if not already assigned, do it and return true
  *pp = pTag;

  return true;
}
