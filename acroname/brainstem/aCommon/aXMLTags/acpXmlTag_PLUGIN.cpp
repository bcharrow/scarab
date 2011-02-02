/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PLUGIN.cpp 		 	                   //
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

#include "acpXmlTag_PLUGIN.h"

#include "acpTag_PLUGIN.h"
#include "acpTag_INSTANCE.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

PLUGIN::~PLUGIN()
{
  while (!m_params.isEmpty())
    m_params.removeHead();
}


/////////////////////////////////////////////////////////////////////

void PLUGIN::addToken(
  const aToken* pToken
)
{
  if (pToken->eType == tkIdentifier) {
    m_pluginName += pToken->v.identifier;
  } else {
    tokenError(pToken, "PLUGIN expects an identifier");
  }	
}


/////////////////////////////////////////////////////////////////////

bool PLUGIN::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "NAME")) {  	
    m_pName = (NAME*)pTag;
    return true;
  } else if (!aStringCompare(pTagName, "VISIBLE")) {
    m_pVisible = (VISIBLE*)pTag;
    return true;
  } else if (!aStringCompare(pTagName, "TRANSLATION")) {
    m_pTranslation = (TRANSLATION*)pTag;
    return true;
  } else if (!aStringCompare(pTagName, "PARAMETER")) {
    m_params.add((PARAMETER*)pTag);
    return true;
  }

  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

char*  PLUGIN::getObjectName()
{ 
  if (m_pName)
    return (char*)m_pName->m_name;
  return NULL;
}


/////////////////////////////////////////////////////////////////////

void PLUGIN::traverse()
{
  acpTag_PLUGIN* pPluginTag = new acpTag_PLUGIN();
  acpList<acpParameter> parameters;
  const char* pName = "";
  if (m_pName)
    pName = m_pName->m_name;
  
  // the translation
  acpVec3 translation;
  if (m_pTranslation)
    translation = acpVec3(m_pTranslation->m_x, 
    			  m_pTranslation->m_y,
    			  m_pTranslation->m_z);

  // collect up the parameters  
  PARAMETER* pParamTag;
  acpListIterator<PARAMETER> paramTags(m_params);
  while ((pParamTag = paramTags.next())) {
    acpParameter* pParam = new acpParameter(pParamTag->m_pName->m_name, 
				            pParamTag->m_pValue);
    parameters.add(pParam);
  }
  pPluginTag->setData(m_pluginName,
  		      m_pImporter->m_nCurrentOwner,
  		      pName,
  		      translation,
  		      parameters);

  m_pImporter->addTag(this, pPluginTag);

  // follow the dynamic tag with the geometries it owns
  if (m_pVisible) {
    aShort saveID = m_pImporter->m_nCurrentOwner;
    m_pImporter->m_nCurrentOwner = pPluginTag->getID();
    GEOMETRY* pGeometry;
    acpListIterator<GEOMETRY> geometries(m_pVisible->m_geometry);
    while ((pGeometry = geometries.next())) {
      m_pImporter->m_nInstanceFlags |= kInstance_Visible;
      pGeometry->traverse();
      m_pImporter->m_nInstanceFlags &= ~kInstance_Visible;
    }
    m_pImporter->m_nCurrentOwner = saveID;
  }
}
