/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpValue.h                                                //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API value object.         //
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

#ifndef _acpValue_H_
#define _acpValue_H_

#include "aOSDefs.h"
#include "aAssert.h"
#include "acpCallback.h"

class acpObject;

class acpValue {

  public:
  			
  			acpValue();
  			acpValue(const acpValue& value);
  			acpValue(const acpValue* pValue);
  			acpValue(const int nValue);
  			acpValue(const float fValue);
  			acpValue(const char* pString);
  			acpValue(const bool bVal);
  			acpValue(const void* vPtr);
  			acpValue(const acpCallback* pcCallback);
  			acpValue(const acpObject* pcObject);
    virtual    		~acpValue();

    typedef enum {
      kInt = 0,
      kFloat,
      kString,
      kBoolean,
      kVoidPtr,
      kCallbackPtr,
      kObjectPtr,
      kEmpty
    } eType;

    void		set(const int nValue);
    void		set(const float fValue);
    void		set(const char* pString);
    void		set(const bool bVal);
    void		set(const void* vp);
    void		set(const acpCallback* pcCallback);
    void		set(const acpObject* pcObject);
    void		set(const acpValue* pValue);

    eType		getType() const
			{ return m_eType; }

    int			getIntVal() const
    			  { aAssert(m_eType == kInt);
    			    return m_v.i; }

    float		getFloatVal() const;

    const char*		getStringVal() const;

    bool		getBoolVal() const
    			  { aAssert(m_eType == kBoolean);
    			    return m_v.b; }

    void*		getVoidPtrVal() const
    			  { if (m_eType == kVoidPtr)
    			      return (void*)m_v.v;
    			    else
    			      return NULL;
    			  }

    acpCallback*	getCallbackPtrVal() const
    			  { if (m_eType == kCallbackPtr)
    			      return m_v.cb;
    			    else
    			      return NULL;
    			  }

    acpObject*		getObjectPtrVal() const
    			  { if (m_eType == kObjectPtr)
    			      return m_v.ob;
    			    else
    			      return NULL;
    			  }

  private:  
    void		empty();

    eType		m_eType;
    union {
      int		i;
      float		f;
      char*		s;
      bool		b;
      const void*	v;
      acpCallback*	cb;
      acpObject*	ob;
    } m_v;
};

#endif // _acpValue_H_

