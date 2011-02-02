/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotObject.h                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotShell client object.       //
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

#ifndef _acpRobotObject_H_
#define _acpRobotObject_H_

#include "acpString.h"
#include "acpList.h"
#include "acpRobotProperty.h"

class acpRobotShell;

class acpRobotObject {
  public:
  				acpRobotObject(
  				  acpRobotShell* pShell,
  				  const char* pClassName,
  			  	  const char* pName) :
  			  	  m_pShell(pShell),
  			  	  m_classname(pClassName),
  			  	  m_name(pName),
  			  	  m_ID(0),
  			  	  m_bPropertiesEnumerated(false),
  			  	  m_bObjectsEnumerated(false),
  			  	  m_pParent(NULL)
  			  	{}
  			  	~acpRobotObject();

    void			setID(
    				  const int ID) 
    				  { m_ID = ID; }
    int				getID() const
    				  { return m_ID; }

    char*			getName() const
    				  { return (char*)m_name; }

    void			addChild(
    				  acpRobotObject* pChild) 
    				  { pChild->m_pParent = this;
    				    m_children.add(pChild); }
    void			addProperty(
    				  acpRobotProperty* pProperty) 
    				  { pProperty->m_pOwner = this;
    				    m_properties.add(pProperty); }

    acpRobotObject*		getParent() const 
    				  { return m_pParent; }

    void			ensurePropertiesEnumerated();
    void			ensureObjectsEnumerated();

    acpRobotProperty*		getProperty(
    				  const char* pName);
    acpRobotObject*		getObject(
    				  const char* pName);

  private:
    acpRobotShell*		m_pShell;

    acpString			m_classname;
    acpString			m_name;
    int				m_ID;
    bool			m_bPropertiesEnumerated;
    bool			m_bObjectsEnumerated;
 
    acpRobotObject*		m_pParent;
    acpList<acpRobotObject> 	m_children;
    acpList<acpRobotProperty> 	m_properties;
    
    friend class		acpRobotProperty;
    friend class		acpRobotShell;
};


#endif // _acpRobotObject_H_
