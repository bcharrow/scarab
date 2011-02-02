/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_CONNECTOR.h 		 		           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of XML tag class.                       //
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

#ifndef _acpXmlTag_CONNECTOR_H_
#define _acpXmlTag_CONNECTOR_H_

#include "acpPkgXML.h"

#include "acpXmlTag_PORT.h"
#include "acpXmlTag_NAME.h"


class CONNECTOR :
  public acpPkgXML
{
  public:
  			CONNECTOR(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			  acpPkgXML(pImporter, pParent),
  			  m_pName(NULL),
  			  m_pPort(NULL)
  			  {}
  			~CONNECTOR()
  			  {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new CONNECTOR(m_pImporter, 
    			  		         pParent); }

    bool		addTag(
    			  acpPkgXML* pTag);

    const char*  	tagName() const
  			  { return "CONNECTOR"; }

    NAME*		m_pName;
    PORT*		m_pPort;
};

#endif // _acpXmlTag_CONNECTOR_H_
