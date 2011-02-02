/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_ROW.cpp 		 		           //
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

#include "acpXmlTag_ROW.h"

#include "acpXmlTag_VIEWS.h"



/////////////////////////////////////////////////////////////////////

bool ROW::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "ROW")
   || !aStringCompare(pTagName, "COLUMN")
   || !aStringCompare(pTagName, "VIEWFRAME"))
    return true;
  
  return false;
}


/////////////////////////////////////////////////////////////////////

void ROW::traverse()
{
  // look up the tree to find the view parent
  VIEWS* pViews = (VIEWS*)findAncestor("VIEWS");
  aAssert(pViews);
  aAssert(pViews->m_buffer);

  acpByte R('R');
  R.writeToStream(pViews->m_buffer);
  
  acpPkgXML::traverse();

  acpByte p('p');
  p.writeToStream(pViews->m_buffer);
}
