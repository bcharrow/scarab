/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_DYNAMIC.h 		 		           //
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

#ifndef _acpXmlTag_DYNAMIC_H_
#define _acpXmlTag_DYNAMIC_H_

#include "acpPkgXML.h"

#include "acpXmlTag_BOUNCE.h"
#include "acpXmlTag_CFM.h"
#include "acpXmlTag_ERP.h"
#include "acpXmlTag_FRICTION.h"
#include "acpXmlTag_MASS.h"
#include "acpXmlTag_NAME.h"
#include "acpXmlTag_TRANSLATION.h"
#include "acpXmlTag_ROTATION.h"
#include "acpXmlTag_VISIBLE.h"


class DYNAMIC :
  public acpPkgXML
{
  public:
  			DYNAMIC(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			    acpPkgXML(pImporter, pParent),
  			    m_pName(NULL),
  			    m_pTranslation(NULL),
  			    m_pRotation(NULL),
  			    m_pMass(NULL),
			    m_pFriction(NULL),
			    m_pSoftERP(NULL),
			    m_pSoftCFM(NULL),
			    m_pBounce(NULL),
  			    m_pVisible(NULL)
    			  {}
  			~DYNAMIC()
			  {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new DYNAMIC(m_pImporter, 
    			  		       pParent); }
    virtual char*	getObjectName() 
    			  { return m_pName ? (char*)m_pName->m_name : 
			                     (char*)NULL; }

    bool		addTag(
    			  acpPkgXML* pTag);

    const char*  	tagName() const
  			  { return "DYNAMIC"; }

    void		traverse();

  private:
    NAME*		m_pName;
    TRANSLATION*	m_pTranslation;
    ROTATION*		m_pRotation;
    MASS*		m_pMass;
    FRICTION*		m_pFriction;
    ERP*		m_pSoftERP;
    CFM*		m_pSoftCFM;
    BOUNCE*		m_pBounce;
    VISIBLE*		m_pVisible;
};

#endif // _acpXmlTag_DYNAMIC_H_
