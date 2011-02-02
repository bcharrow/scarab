/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_GEOMETRY.h 		 		           //
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

#ifndef _acpXmlTag_GEOMETRY_H_
#define _acpXmlTag_GEOMETRY_H_

#include "acpShort.h"
#include "acpPackageTag.h"
#include "acpPkgXML.h"


class GEOMETRY :
  public acpPkgXML {
  public:
  			GEOMETRY(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			  acpPkgXML(pImporter, pParent),
			  m_pGeometry(NULL),
			  m_pName(NULL)
  			  {}

    virtual acpPkgXML*	clone(
    			  acpPkgXML* pParent = NULL)
    			    { return new GEOMETRY(m_pImporter,
    			    			  pParent); }
    virtual bool	equalGeometry(
    			  const acpPkgXML* pTag) const;

    virtual void	addGeometry() 
    			  { aAssert(0); }

    virtual bool	addTag(
    			  acpPkgXML* pTag);

    virtual const char* tagName() const
    			  { return "GEOMETRY"; }

    acpPkgXML*		parent() const
    			  { return m_pParent; }

    virtual void	writeToStream(
    			  aStreamRef stream) const
    			  {}
    
    const acpList<acpShort>*  getIDs() const
    			  { return &m_tagIndices; }
    			  
    void		addPackageTag(
    			  acpPackageTag* pTag);

    void		traverse();

  protected:
    acpList<acpShort>	m_tagIndices;

  private:
    acpPkgXML*		m_pGeometry;
    acpPkgXML*		m_pName;

  friend class		acpPkgImport;
};

#endif // _acpXmlTag_GEOMETRY_H_
