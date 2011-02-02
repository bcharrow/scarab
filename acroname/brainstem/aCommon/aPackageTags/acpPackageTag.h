/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPackageTag.h                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of virtual robot data package       	   //
//		tag structure.					   //
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

#ifndef _acpPackageTag_H_
#define _acpPackageTag_H_

#include "aIO.h"
#include "acpList.h"


/////////////////////////////////////////////////////////////////////

class acpPackageTag {
  public:
  				acpPackageTag(
  				  const aShort tagType,
  				  void* pRef = NULL) :
				  m_pRef(pRef),
  				  m_tagType(tagType),
  				  m_nID(0),
  				  m_nOwnerID(0)
  				  {}

    virtual			~acpPackageTag() {}
    				  
    aShort			getTagType() const
    				  { return m_tagType; }

    const char*			getTagTypeName() const;

    virtual acpPackageTag*	factoryFromStream(
    				  aStreamRef stream) = 0;

    aInt32			writeToStream(
    				  aStreamRef stream) const;

    void*			getRef()
    				  { return m_pRef; }

    void			setRef(
    				  void* pRef)
    				  { m_pRef = pRef; }
    
    aShort			getID() const
    				  { return m_nID; }
    				  
    void			setID(
    				  const aShort nID)
    				  { m_nID = nID; }

    aShort			getOwnerID() const
    			  	  { return m_nOwnerID; }
 
    void                        setOwnerID(
    				  const aShort nOwnerID)
    				  { m_nOwnerID = nOwnerID; }

  protected:
    virtual void		writeData(
    				  aStreamRef stream) const = 0; 

    void*			m_pRef;

  private:
    aShort			m_tagType;
    aShort			m_nID;
    aShort			m_nOwnerID;
};

#endif // _acpPackageTag_H_
