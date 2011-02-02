/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaUserObj.h                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API user object.          //
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

#ifndef _acpGarciaUserObj_H_
#define _acpGarciaUserObj_H_

#include "acpObject.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpGarciaInternal.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the servo class

#define aGUP_MODULE			"module"
#define aGUP_SIZE			"size"
#define aGUP_FILTER_BYTE		"filter-byte"
#define aGUP_TIMEOUT			"timeout"
#define aGUP_DATA_PTR			"data-ptr"
#define aGUP_RESULT			"result"



class acpGarciaInternal;
class acpGarciaUserObjProperty;

class acpGarciaUserObj :
  public acpObject
{
  protected:
  				acpGarciaUserObj(
				  const char* pName,
  				  acpGarciaInternal* pcGarciaInternal);

  public:
    virtual   			~acpGarciaUserObj();

    virtual aErr		writeToStream(
    				  const aStreamRef stream) const;

    virtual void		getDescription(acpTextBuffer& buffer) {}

  protected:

    char			m_cModule;
    char			m_cSize;
    char			m_cFilterByte;
    unsigned long		m_ulTimeout;
    char*			m_pData;
    acpGarciaInternal*		m_pcGarciaInternal;

  friend class acpGarciaInternal;
  friend class acpGarciaUserObjProperty;
  friend class acpGarciaUserObjModuleProperty;
  friend class acpGarciaUserObjSizeProperty;
  friend class acpGarciaUserObjFilterByteProperty;
  friend class acpGarciaUserObjTimeoutProperty;
  friend class acpGarciaUserObjDataPtrProperty;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaUserObjProperty : public acpProperty
{
  public:
  				acpGarciaUserObjProperty(
  				  acpGarciaUserObj* pcGarciaUserObj,
  				  const char* pName,
  				  aPROPERTY_FLAGS flags);

    virtual			~acpGarciaUserObjProperty() {}

  protected:

    void			sendUserObjPacket();

    int				doUserObjAction();


  protected:

    acpGarciaUserObj*		m_pcGarciaUserObj;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaUserObjModuleProperty : public acpGarciaUserObjProperty
{
  public:
  				acpGarciaUserObjModuleProperty(
  				  acpGarciaUserObj* pcGarciaUserObj);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaUserObjSizeProperty : public acpGarciaUserObjProperty
{
  public:
  				acpGarciaUserObjSizeProperty(
  				  acpGarciaUserObj* pcGarciaUserObj);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaUserObjFilterByteProperty : public acpGarciaUserObjProperty
{
  public:
  				acpGarciaUserObjFilterByteProperty(
  				  acpGarciaUserObj* pcGarciaUserObj);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaUserObjTimeoutProperty : public acpGarciaUserObjProperty
{
  public:
  				acpGarciaUserObjTimeoutProperty(
  				  acpGarciaUserObj* pcGarciaUserObj);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaUserObjDataPtrProperty : public acpGarciaUserObjProperty
{
  public:
  				acpGarciaUserObjDataPtrProperty(
  				  acpGarciaUserObj* pcGarciaUserObj);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};

/////////////////////////////////////////////////////////////////////

class acpGarciaUserObjResultProperty : public acpGarciaUserObjProperty
{
  public:
  				acpGarciaUserObjResultProperty(
  				  acpGarciaUserObj* pcGarciaUserObj);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};

#endif // _acpGarciaUserObj_H_
