/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotUserObjects.h                                     //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the user object template             //
//              and user object instance classes.                  //
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

#ifndef _acpRobotUserObjects_H_
#define _acpRobotUserObjects_H_

#include "acpObject.h"
#include "acpTag_TEMPLATE.h"
#include "acpTag_USEROBJECT.h"

#include "acpRobotPackage.h"
#include "acpRobotProperties.h"
#include "acpRobotInternal.h"

class acpRobotBaseObject:
  public acpObject
{
  public:
				acpRobotBaseObject(
				  acpRobotInternal* pcRobotInternal,
				  acpTag_TEMPLATE* pTemplateTag);

    virtual 			~acpRobotBaseObject();

    int				getBufferSize() const
				{ return m_nBufferSize; }
    int				getUserPropCt() const
				{ return m_nUserPropCt; }

  protected: 

    int				addBaseProperty(
    				  acpRobotUserProperty* pProperty,
    				  acpValue* pDefault);

    acpRobotInternal*		m_pcRobotInternal;  

    int				m_nUserPropCt;
    int				m_nBufferSize;

  friend class acpRobotInternal;
  friend class acpRobotUserProperty;
};

class acpRobotUserObject:
  public acpObject
{
  public:
				acpRobotUserObject(
				  acpRobotInternal* pcRobotInternal,
				  acpTag_USEROBJECT* pObjTag);

    virtual 			~acpRobotUserObject();

    void			setTemplate(
				  acpRobotBaseObject* pTemplate);
    acpRobotBaseObject*		getTemplate()
				{ return m_pcTemplate; }

    void			initialize(
				  const int nPropIndex);

    virtual acpValue*		getNamedValue(
    				  const char* pPropName);
    virtual acpValue*		getValue(
				  const int nPropIndex);

    virtual void		setNamedValue(
                                  const char* pPropName,
    				  const acpValue* pValue);
    virtual void		setValue(
				  const int nPropIndex,
    				  const acpValue* pValue);

  protected: 

    acpRobotInternal*		m_pcRobotInternal;  
    acpRobotBaseObject*		m_pcTemplate;

    char*			m_pData;
    int				m_nDataBytes;
    
    int				m_nLinkID;
    
  friend class acpRobotInternal;
  friend class acpRobotUserProperty;
};

#endif // _acpRobotUserObjects_H_
