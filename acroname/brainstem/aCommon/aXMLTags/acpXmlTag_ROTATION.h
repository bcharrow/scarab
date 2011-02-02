/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_ROTATION.h 		 		           //
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

#ifndef _acpXmlTag_ROTATION_H_
#define _acpXmlTag_ROTATION_H_

#include "acpPkgXML.h"


class ROTATION :
  public acpPkgXML
{
  public:
  			ROTATION(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			    acpPkgXML(pImporter, pParent),
  			    m_count(0),
  			    m_sign(1),
  			    m_a(0),
  			    m_x(0),
  			    m_y(0),
  			    m_z(0)
  			    {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new ROTATION(m_pImporter, 
    			  		        pParent); }

    void		addToken(
    			  const aToken* pToken);

    const char*  	tagName() const
  			  { return "ROTATION"; }

    bool		getDescription(
    			  acpString& description);
    bool		getDirectFormat(
    			  acpString& format,
    			  acpString& example,
    			  acpString& exampleNotes);

  private:
    int			m_count;
    int			m_sign;
    aFloat		m_a;
    aFloat		m_x;
    aFloat		m_y;
    aFloat		m_z;

    friend class	TRANSFORM;
    friend class	STATIC;
    friend class	DYNAMIC;
    friend class	FIXEDCAMERA;
};

#endif // _acpXmlTag_ROTATION_H_
