/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_VRML2.h 	 		                   //
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

#ifndef _acpXmlTag_VRML2_H_
#define _acpXmlTag_VRML2_H_

#include "acpPkgXML.h"

#include "acpXmlTag_GEOMETRY.h"


class VRML2 :
  public GEOMETRY
{
  public:
  			VRML2(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			    GEOMETRY(pImporter, pParent)
  			    {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new VRML2(m_pImporter, 
    			  		     pParent); }

    bool		equalGeometry(
    			  const acpPkgXML* pTag) const;

    void		addGeometry();
 
    void		addToken(
    			  const aToken* pToken);

    const char*  	tagName() const
  			  { return "VRML2"; }

    bool		getDescription(
    			  acpString& description);
    bool		getDirectFormat(
    			  acpString& format,
    			  acpString& example,
    			  acpString& exampleNotes);

  private:
    acpString		m_vrmlFileName;
};

#endif // _acpXmlTag_VRML2_H_
