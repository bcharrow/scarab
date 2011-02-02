/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_VIEWFRAME.cpp 		                   //
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

#include "acpXmlTag_VIEWFRAME.h"

#include "acpXmlTag_TITLE.h"
#include "acpXmlTag_WIDTH.h"
#include "acpXmlTag_HEIGHT.h"
#include "acpXmlTag_VIEWS.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

bool VIEWFRAME::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "TITLE")) {
    m_title = ((TITLE*)pTag)->m_title;

  } else if (!aStringCompare(pTagName, "WIDTH")) {
    aInt32 val = *((WIDTH*)pTag);
    if ((val < 0) || (val > 255))
      m_pImporter->parseError("VIEWFRAME WIDTH values must range from 0-255");
    else
      m_width = (aByte)val;

  } else if (!aStringCompare(pTagName, "HEIGHT")) {
    aInt32 val = *((HEIGHT*)pTag);
    if ((val < 0) || (val > 255))
      m_pImporter->parseError("VIEWFRAME HEIGHT values must range from 0-255");
    else
      m_height = (aByte)val;

  // allow GL sub view
  } else if (!aStringCompare(pTagName, "OPENGLVIEW")
  	     || !aStringCompare(pTagName, "OSCILISCOPE")
  	     || !aStringCompare(pTagName, "CONTROLVIEW")) {
    m_pSubView = pTag;

  } else {
    return false;
  }

  return true;
}


/////////////////////////////////////////////////////////////////////

void VIEWFRAME::traverse()
{
  VIEWS* pViews = (VIEWS*)findAncestor("VIEWS");
  aAssert(pViews);

  acpByte F('F');
  F.writeToStream(pViews->m_buffer);

  m_width.writeToStream(pViews->m_buffer);

  m_height.writeToStream(pViews->m_buffer);

  m_title.writeToStream(pViews->m_buffer);
  
  if (m_pSubView)
    m_pSubView->traverse();

  acpByte p('p');
  p.writeToStream(pViews->m_buffer);
}
