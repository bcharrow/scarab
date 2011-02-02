/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PROPERTY.h 	 		                   //
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

#ifndef _acpXmlTag_PROPERTY_H_
#define _acpXmlTag_PROPERTY_H_

#include "acpPkgXML.h"
#include "aRobotDefs.tea"


class PROPERTY :
  public acpPkgXML
{
  public:
  			PROPERTY(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			  acpPkgXML(pImporter, pParent),
  			  m_unittype(aROBOT_UNITS_NONE),
  			  m_unitcode(aROBOT_UNITS_DEFAULT),
  			  m_pSetTag(NULL),
  			  m_pGetTag(NULL)
  			  {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new PROPERTY(m_pImporter, 
    			  		     pParent); }
    const char*  	tagName() const
  			  { return "PROPERTY"; }

    bool		addTag(
    			  acpPkgXML* pTag);

    virtual void	traverse();

  private:
    acpString		m_name;    
    acpString		m_linkname;
    acpString		m_description;
    acpString		m_type;
    acpString		m_default;
    aByte		m_unittype;
    aByte		m_unitcode;
    acpPkgXML*		m_pSetTag;
    acpPkgXML*		m_pGetTag;
};

#endif // _acpXmlTag_PROPERTY_H_
