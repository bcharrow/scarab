/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPackage.h	 	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of virtual robot data package       	   //
//		routines.					   //
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

#ifndef _acpPackage_H_
#define _acpPackage_H_

#include "aIO.h"
#include "acpString.h"
#include "acpList.h"
#include "acpPackageTag.h"


/////////////////////////////////////////////////////////////////////

class acpPackageProgress {
  public:		acpPackageProgress() 
                          {}
    virtual		~acpPackageProgress()
                          {}
  
  virtual void          updateProgress(
  			  const aInt32 nTotal,
  			  const aInt32 nCurrent) 
  			  {}
};


/////////////////////////////////////////////////////////////////////

class acpPackage
{
  public:
	 	 		acpPackage(
	 	 		  acpPackageProgress* pProgress,
	 	 		  aIOLib ioRef,
	 	 		  aByte versionMajor,
	 	 		  aByte versionMinor,
	 	 		  aInt32 build,
	 	 		  aShort fileRev,
	 	 		  aStreamRef stream = NULL);
    virtual			~acpPackage();

    virtual void		initialize() = 0;
                                  
    void			addRecognizedTag(
    				  acpPackageTag* pTag)
    				  { m_recognizedTags.add(pTag); }

    acpPackageTag*		consumeHeadTag()
    				  { return m_tags.removeHead(); }

    aInt32			numTags() const
    				  { return m_tags.length(); }

    aInt32			getTagChild(
				  const aInt32 nTagID) const;

    void			addTag(
    				  acpPackageTag* pTag);

    void			writeToStream(
    				  aStreamRef stream);

    acpPackageTag*		findTagByType(
    				  const aShort nType);

    aInt32			getBuild() const 
    			          { return m_build; }

  protected:
    acpPackageProgress* 	m_progressIndicator;
    aStreamRef			m_initStream;

    static const char*		m_signature;

    aByte			m_versionMajor;
    aByte			m_versionMinor;
    aInt32			m_build;
    aShort			m_fileRev;

    aIOLib			m_ioRef;
    acpList<acpPackageTag>	m_recognizedTags;
    acpList<acpPackageTag>	m_tags;
};

#endif // _acpPackage_H_
