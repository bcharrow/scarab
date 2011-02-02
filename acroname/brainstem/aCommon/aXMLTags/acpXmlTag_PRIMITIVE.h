/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PRIMITIVE.h 	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of XML tag class.                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// Copyright 1994-2008. Acroname Inc.                              //
//                                                                 //
// This software is the PRIMITIVE of Acroname Inc.  Any             //
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

#ifndef _acpXmlTag_PRIMITIVE_H_
#define _acpXmlTag_PRIMITIVE_H_

#include "acpPkgXML.h"


class PRIMITIVE :
  public acpPkgXML
{
  public:
  			PRIMITIVE(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			  acpPkgXML(pImporter, pParent),
  			  m_nPropCt(0)
  			  {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new PRIMITIVE(m_pImporter, 
    			  		     pParent); }
    const char*  	tagName() const
  			  { return "PRIMITIVE"; }

    bool		addTag(
    			  acpPkgXML* pTag);

    virtual void	traverse();

  private:
    acpString		m_name;
    acpString		m_linkname;
    acpString		m_description;
    acpString		m_apifile;
    acpString		m_stemfile;
    aShort		m_nPropCt;
};

#endif // _acpXmlTag_PRIMITIVE_H_
