/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpObject.h 		  		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of Pure Virtual Base Property Object    //
//              Class Definition.                                  //
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

#ifndef _acpObject_H_
#define _acpObject_H_

#include "aIO.h"

#include "acpProperty.h"
#include "acpList.h"

typedef aErr (*aObjectEnumProc)(acpObject& object,
				void* vpRef);

class acpObject 
{
  public:
  				acpObject(
  				  const char* pClassName,
  				  const char* pName = NULL);
  				acpObject(
  				  aStreamRef stream) {}
    virtual			~acpObject();

    virtual int			numSubObjects() const;
    virtual void		enumSubObjects (
    				  aObjectEnumProc enumProc,
    				  void* vpRef,
    				  const char* pClassNameFilter = NULL) const;
    virtual acpObject*		getNamedSubObject (
    				  const char* pClassName,
    				  const char* pName) const;
    virtual acpObject*		getSubObject (
    				  const int nObjectIndex) const;

    virtual int			numProperties() const;
    virtual void		enumProperties (
    				  propertyEnumProc enumProc,
    				  void* vpRef) const;
    virtual acpProperty*	getProperty(
    				  const int nPropertyIndex) const;
    virtual int			getPropertyIndex(
    				  const char* pPropertyName) const;
    virtual char*		getPropertyName(
    				  const int nPropertyIndex) const;
    virtual aPROPERTY_FLAGS	getPropertyFlags(
    				  const int nPropertyIndex) const;

    virtual acpValue*		getValue(
    				  const int nPropIndex);
    virtual acpValue*		getNamedValue(
    				  const char* pPropName);

    virtual void		setValue(
    				  const int nPropIndex,
    				  const acpValue* pValue);
    virtual void		setNamedValue(
                                  const char* pPropName,
    				  const acpValue* pValue);

    virtual aErr		readValue(
                                  const int nPropIndex,
    				  aStreamRef stream);

    int				m_nClassPropertyIndex;
    int				m_nNamePropertyIndex;

  protected:
    int				addProperty(
    				  acpProperty* pProperty,
    				  acpValue* pDefault = NULL);
    void			addSubObject(
    				  acpObject* pObject);

  private:
    acpList<acpObject>*		m_pSubObjects;
    acpList<acpValue>*		m_pValues;
    acpList<acpProperty>*	m_pProperties;

    int				m_nIndex;
  
  friend class acpProperty;
};


#endif // _acpObject_H_
