/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_OPENGLVIEW.h 		 		                   //
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

#ifndef _acpXmlTag_OPENGLVIEW_H_
#define _acpXmlTag_OPENGLVIEW_H_

#include "acpPkgXML.h"

#include "acpXmlTag_FOV.h"
#include "acpXmlTag_TRANSPARENCY.h"

class OPENGLVIEW :
  public acpPkgXML
{
  public:
  			OPENGLVIEW(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			    acpPkgXML(pImporter, pParent),
  			    m_pCamera(NULL),
  			    m_pFOV(NULL),
  			    m_pTransparency(NULL),
  			    m_nFlags(0)
  			    {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new OPENGLVIEW(m_pImporter, 
    			  			  pParent); }

    bool		addTag(
    			  acpPkgXML* pTag);

    const char*  	tagName() const
  			  { return "OPENGLVIEW"; }

    virtual void	traverse();

  private:
    acpPkgXML* 		m_pCamera;
    FOV*		m_pFOV;
    TRANSPARENCY*	m_pTransparency;
    int			m_nFlags;
};

#endif // _acpXmlTag_OPENGLVIEW_H_
