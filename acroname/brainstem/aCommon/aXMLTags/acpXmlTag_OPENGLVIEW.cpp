/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_OPENGLVIEW.cpp 		                   //
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

#include "acpGLView.h"
#include "acpXmlTag_OPENGLVIEW.h"

#include "acpXmlTag_VIEWS.h"



/////////////////////////////////////////////////////////////////////

bool OPENGLVIEW::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "OBSERVECAMERA")) {
    aAssert(m_pCamera == NULL);
    m_pCamera = pTag;
    return true;
  } else if (!aStringCompare(pTagName, "FIXEDCAMERA")) {
    aAssert(m_pCamera == NULL);
    m_pCamera = pTag;
    return true;
  } else if (!aStringCompare(pTagName, "TRANSPARENCY")) {
    m_pTransparency = (TRANSPARENCY*)pTag;
    return true;
  } else if (!aStringCompare(pTagName, "FOV")) {
    m_pFOV = (FOV*)pTag;
    return true;
  }
  
  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

void OPENGLVIEW::traverse()
{
  VIEWS* pViews = (VIEWS*)findAncestor("VIEWS");
  aAssert(pViews);

  acpByte b('G');
  b.writeToStream(pViews->m_buffer);

  acpFloat FOV(45.0f);
  if (m_pFOV)
    FOV = (float)*m_pFOV;
  FOV.writeToStream(pViews->m_buffer);

  if (m_pTransparency && m_pTransparency->m_value)
    m_nFlags |= aGLViewFlagTransparency;

  acpByte flags((aByte)m_nFlags);
  flags.writeToStream(pViews->m_buffer);

  if (m_pCamera)
    m_pCamera->traverse();
}
