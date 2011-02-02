/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotProperty.h                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotShell client Property.     //
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

#ifndef _acpRobotProperty_H_
#define _acpRobotProperty_H_

#include "acpString.h"
#include "acpList.h"

class acpRobotObject;

class acpRobotProperty {
  public:
  				acpRobotProperty(
  			  	  const char* pName,
  			  	  const int nIndex,
  			  	  const int nTypeFlags) :
  			  	  m_name(pName),
  			  	  m_index(nIndex),
  			  	  m_typeFlags(nTypeFlags),
  			  	  m_bDescriptionCached(false),
  			  	  m_pOwner(NULL)
  			  	{}
  			  	~acpRobotProperty();

    int				getIndex() const
    				  { return m_index; }
    int				getTypeFlags() const
    				  { return m_typeFlags; }

    const char*			getName() const
    				  { return (char*)m_name; }

    const char*			getDescription();

    void			setOwner(
    				  acpRobotObject* pOwner) 
    				  { m_pOwner = pOwner; }
    acpRobotObject*		getOwner() const 
    				  { return m_pOwner; }

    void			set(); 
    void			get();

  private:
    int				getNegInt();
    float			getNegFloat();

    acpString			m_name;
    int				m_index;
    int				m_typeFlags;
    
    bool                        m_bDescriptionCached;
    acpString                   m_description;
 
    acpRobotObject*		m_pOwner;
    
    friend class		acpRobotObject;
};


#endif // _acpRobotProperty_H_
