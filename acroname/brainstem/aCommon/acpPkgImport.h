/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPkgImport.h 	  		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of virtual robot data package routines. //
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

#ifndef _acpPkgImport_H_
#define _acpPkgImport_H_

#include "aIO.h"
#include "acpList.h"
#include "acpPackage.h"
#include "acpPkgXML.h"


/////////////////////////////////////////////////////////////////////

class acpPkgImport {

  public:
				acpPkgImport(
				  aIOLib ioRef,
				  aStreamRef logStream,
				  aStreamRef input,
				  acpPackage* pPackage);
    virtual			~acpPkgImport();

    virtual void		fillInTags() = 0;

    void			addRecognizedTag(
    				  acpPkgXML* pTag)
    				  { m_tags.add(pTag); }

    virtual void		import(
				  aStreamRef input);

    void			error(
				  const char* pMsg);
    void			log(
				  const char* pMsg);

    static aErr 		xmlErr(
    				  tkError error,
			          const unsigned int nLine,
			          const unsigned int nColumn,
			          const unsigned int nData,
			          const char* data[],
			          void* errProcRef);

    static aErr 		sStart(
    				  aXMLNodeRef node,
	     	                  const char* pKey,
	     	                  void* vpRef);

    static aErr 		sContent(
    				   aXMLNodeRef node,
				   const char* pKey,
	                 	   const aToken* pValue,
	                	   void* vpRef);

    static aErr 		sEnd(
    				  aXMLNodeRef node,
		    	          aXMLNodeRef parent,
		      	          const char* pKey,
		  	          void* vpRef);

    void			parseError(
    				  const char* pMsg);

    int				numErrors() const
    				  { return m_nImportErrors; }

    virtual void		addTag(
  				  acpPkgXML* pXMLOrigin,
    				  acpPackageTag* pTag);

    acpPackageTag*		getNamedTag(
    				  const char* pName);

    aStreamRef			getLogStream()
    				  { return m_logStream; }
    aStreamRef			getErrorStream()
    				  { return m_logStream; }

    aInt32			numPackageTags() const
    				  { return m_pPackage->numTags(); }

    aShort			m_nCurrentOwner;
    aByte			m_nInstanceFlags;
    
    aShort			getTagChild(
				  const aShort nTagID) const
				  { return (aShort)m_pPackage->getTagChild(nTagID); }

  protected:
    acpPackage*			m_pPackage;
    acpList<acpPkgXML>		m_tags;
    aIOLib			m_ioRef;
  
  private:
    aStreamRef			m_logStream;
    aStreamRef			m_input;
    aXMLRef			m_xml;

    // xml management
    int				m_nImportErrors;
    int				m_nPackageNesting;
    acpList<acpPkgXML>		m_stack;

    // named reference managment
    aSymbolTableRef		m_namedTags;
    static aErr 		symDontDelete(
    				  void* pData, 
    				  void* ref) 
    				  { return aErrNone; }

  friend class			acpPkgXML;
};

#endif // _acpPkgImport_H_
