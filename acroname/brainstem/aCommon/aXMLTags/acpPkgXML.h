/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPkgXML.h				                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: XML Package Tag class.				   //
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

#ifndef _acpPkgXML_H_
#define _acpPkgXML_H_

#include "aIO.h"
#include "acpByte.h"
#include "acpString.h"
#include "acpFloat.h"
#include "acpList.h"
#include "acpException.h"


/////////////////////////////////////////////////////////////////////

#define tagTokenError	100
#define tagFileError	101

/////////////////////////////////////////////////////////////////////


class acpPkgImport;

class acpPkgXML {
  public:
  			acpPkgXML(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL);
    virtual		~acpPkgXML();

    virtual acpPkgXML*	clone(
    			  acpPkgXML* pParent = NULL) = 0;

    virtual void	addToken(
    			  const aToken* pToken);

    virtual bool	addTag(
    			  acpPkgXML* pTag);

    virtual const char* tagName() const = 0;

    virtual void	end()
    			  {}
    
    virtual char*	getObjectName() 
    			  { return NULL; }

    virtual void	traverse();

    acpPkgXML*		parent() const
    			  { return m_pParent; }

    virtual void	writeToStream(
    			  aStreamRef stream) const
    			  {}

    void		addChild(
    			  acpPkgXML* pTag)
    			  { m_children.add(pTag); }

    acpPkgXML* 		findAncestor(
    			  const char* pTagName) const;

    void		tagError(
    			  const char* pMsg);

    void                setOwnerID(
			  const aShort nOwnerID)
			  { m_nOwnerID = nOwnerID; }

    virtual bool	getDescription(
    			  acpString& description) 
    			  { return false; }

    virtual bool	getDirectFormat(
    			  acpString& format,
    			  acpString& example,
    			  acpString& exampleNotes)
    			  { return false; }

    // fill this in for how this tag affects the
    // parent
    virtual bool	getChildDescription(
    			  acpPkgXML* pParent,
    			  acpString& description) 
    			  { return false; }

    virtual bool	assignExclusive(
			  void* pointer,
    			  acpPkgXML* pTag);

protected:
    void		tokenError(
    			  const aToken* pToken,
    			  const char* pMsg);

  protected:
    aIOLib		m_ioRef;
    acpPkgImport*	m_pImporter;
    acpPkgXML*		m_pParent;
    acpList<acpPkgXML>	m_children;
    aShort		m_nOwnerID;
};

#define aEXASSIGN(p, t) assignExclusive(&p, t)

#endif // _acpPkgXML_H_
