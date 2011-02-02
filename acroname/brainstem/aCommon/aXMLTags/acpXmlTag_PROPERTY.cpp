/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PROPERTY.cpp	 		                   //
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

#include "acpXmlTag_PROPERTY.h"

#include "acpProperty.h"
#include "acpTag_PROPERTY.h"
#include "acpXmlTag_apiNAME.h"
#include "acpXmlTag_TYPE.h"
#include "acpXmlTag_DISTANCEUNIT.h"
#include "acpXmlTag_ANGLEUNIT.h"
#include "acpXmlTag_MASSUNIT.h"
#include "acpXmlTag_DEFAULT.h"
#include "acpXmlTag_DESCRIPTION.h"
#include "acpXmlTag_LINKNAME.h"
#include "acpPkgImport.h"


/////////////////////////////////////////////////////////////////////

bool PROPERTY::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();

  if (!aStringCompare(pTagName, "NAME")) {
    m_name = ((apiNAME*)pTag)->m_name;
  } else if (!aStringCompare(pTagName, "LINKNAME")) {
    if (findAncestor("PRIMITIVE")) {
      tagError("Primitive PROPERTY may not have a LINKNAME");
    } else if (findAncestor("TEMPLATE")) {
      tagError("Template PROPERTY may not have a LINKNAME");
    } else {
      m_linkname = ((LINKNAME*)pTag)->m_name;
    }
  } else if (!aStringCompare(pTagName, "DESCRIPTION")) {
    m_description = ((DESCRIPTION*)pTag)->m_description;
  } else if (!aStringCompare(pTagName, "DISTANCEUNIT")) {
    m_unittype = aROBOT_UNITS_DISTANCE;
    m_unitcode = ((DISTANCEUNIT*)pTag)->m_cUnitCode;
  } else if (!aStringCompare(pTagName, "ANGLEUNIT")) {
    m_unittype = aROBOT_UNITS_ANGLE;;
    m_unitcode = ((ANGLEUNIT*)pTag)->m_cUnitCode;
  } else if (!aStringCompare(pTagName, "MASSUNIT")) {
    m_unittype = aROBOT_UNITS_MASS;;
    m_unitcode = ((MASSUNIT*)pTag)->m_cUnitCode;
  } else if (!aStringCompare(pTagName, "TYPE")) {
    m_type = ((TYPE*)pTag)->m_type;
  } else if (!aStringCompare(pTagName, "DEFAULT")) {
    m_default = ((DEFAULT*)pTag)->m_data;
  } else if (!aStringCompare(pTagName, "SET")) {
    if (m_pSetTag) {
      tagError("can have at most one SET block");
    }
    m_pSetTag = pTag;
  } else if (!aStringCompare(pTagName, "GET")) {
    if (m_pGetTag) {
      tagError("can have at most one GET block");
    }
    m_pGetTag = pTag;
  } else if (!aStringCompare(pTagName, "DESCRIPTION")) {
    m_description += ((DESCRIPTION*)pTag)->m_description;
  } else {
    return false;
  }

  return true;
}


/////////////////////////////////////////////////////////////////////

void PROPERTY::traverse()
{
  // check for a set or get tag
  if (!m_pSetTag && !m_pGetTag) {
    tagError("PROPERTY must have a SET and/or GET child");
  } else {

    acpTag_PROPERTY* pPropTag = new acpTag_PROPERTY();

    aPROPERTY_FLAGS flags = 0;
    if (!aStringCompare(m_type, "int")) {
      flags |= aPROPERTY_FLAG_INT;
    } else if (!aStringCompare(m_type, "float")) {
      flags |= aPROPERTY_FLAG_FLOAT;
    } else if (!aStringCompare(m_type, "bool")) {
      flags |= aPROPERTY_FLAG_BOOL;
    } else if (!aStringCompare(m_type, "string")) {
      flags |= aPROPERTY_FLAG_STRING;
    } else {
      tagError("type not specified for PROPERTY");
    }
    if (m_pSetTag) flags |= aPROPERTY_FLAG_WRITE;
    if (m_pGetTag) flags |= aPROPERTY_FLAG_READ;

    pPropTag->setData(
      m_pImporter->m_nCurrentOwner,
      m_name,
      m_linkname,
      flags,
      m_unittype,
      m_unitcode,
      m_default,
      m_description);

    m_pImporter->addTag(this, pPropTag);

    aShort ID = pPropTag->getID();
    aShort saveID = m_pImporter->m_nCurrentOwner;
    m_pImporter->m_nCurrentOwner = ID;

    // we have a SET and/or GET child
    if (m_pSetTag) {
      m_pSetTag->traverse();
    }
    if (m_pGetTag) {
      m_pGetTag->traverse();
    }

    m_pImporter->m_nCurrentOwner = saveID;
  }
}
