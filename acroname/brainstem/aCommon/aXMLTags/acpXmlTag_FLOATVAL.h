/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_FLOATVAL.h 		 		           //
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

#ifndef _acpXmlTag_FLOATVAL_H_
#define _acpXmlTag_FLOATVAL_H_

#include "acpPkgXML.h"


class FLOATVAL :
  public acpPkgXML
{
  public:
  			FLOATVAL(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
			    acpPkgXML(pImporter, pParent),
			    m_bNeg(false),
			    m_value(0.0f)
  			    {}
			  
    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return NULL; }

    void		addToken(
    			  const aToken* pToken);

    const char*  	tagName() const
  			  { return "FLOATVAL"; }

    bool		getDirectFormat(
    			  acpString& format,
    			  acpString& example,
    			  acpString& exampleNotes);

    operator const	float() const
    			  { return m_value; }

  protected:
    bool                m_bNeg;
    acpFloat		m_value;
};

#endif // _acpXmlTag_FLOATVAL_H_
