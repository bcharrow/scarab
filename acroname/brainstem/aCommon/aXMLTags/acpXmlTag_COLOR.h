/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_COLOR.h 		 		           //
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

#ifndef _acpXmlTag_COLOR_H_
#define _acpXmlTag_COLOR_H_

#include "acpPkgXML.h"


class COLOR :
  public acpPkgXML
{
  public:
  			COLOR(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			  acpPkgXML(pImporter, pParent),
  			  m_count(0)
  			  {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new COLOR(m_pImporter, 
    			  		      pParent); }

    const bool          operator==(
                          const COLOR& rhs) const;

    void		addToken(
    			  const aToken* pToken);

    const char*  	tagName() const
  			  { return "COLOR"; }

  private:
    int			m_count;
    aFloat		m_r;
    aFloat		m_g;
    aFloat		m_b;
  
  friend class		SPHERE;
  friend class		CYLINDER;
  friend class		PLANE;
  friend class		BOX;
};

#endif // _acpXmlTag_COLOR_H_
