/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpValue.cpp                                              //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API value object.     //
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

#include "aOSDefs.h"
#include "acpValue.h"


/////////////////////////////////////////////////////////////////////

acpValue::acpValue()
{
  aBZero(&m_v, sizeof(m_v));
  m_eType = kEmpty;

} // acpValue default constructor


/////////////////////////////////////////////////////////////////////

acpValue::acpValue(
  const acpValue& value
)
{
  m_eType = kEmpty; /* courtesy of Tom Brown */
  set(&value);
}


/////////////////////////////////////////////////////////////////////

acpValue::acpValue(
  const acpValue* pValue
)
{
  m_eType = kEmpty; /* courtesy of Tom Brown */
  set(pValue);
}



/////////////////////////////////////////////////////////////////////

acpValue::acpValue (
  const int nValue
)
{
  m_eType = kInt;
  m_v.i = nValue; 
}



/////////////////////////////////////////////////////////////////////

acpValue::acpValue (
  const float fValue
)
{
  m_eType = kFloat;
  m_v.f = fValue; 
}


/////////////////////////////////////////////////////////////////////

acpValue::acpValue (
  const char* pString
)
{
  m_eType = kString;
  unsigned int len = aStringLen(pString);
  m_v.s = (char*)aMemAlloc((len + 1) * sizeof(char));
  aStringCopySafe(m_v.s, len + 1, pString);
}



/////////////////////////////////////////////////////////////////////

acpValue::acpValue (
  const bool bVal
)
{
  m_eType = kBoolean;
  m_v.b = bVal;
}



/////////////////////////////////////////////////////////////////////

acpValue::acpValue (
  const void* vPtr
)
{
  m_eType = kVoidPtr;
  m_v.v = vPtr;
}


/////////////////////////////////////////////////////////////////////

acpValue::acpValue (
  const acpCallback* pcCallback
)
{
  m_eType = kCallbackPtr;
  m_v.cb = (acpCallback*)pcCallback;
}


/////////////////////////////////////////////////////////////////////

acpValue::acpValue (
  const acpObject* pcObject
)
{
  m_eType = kObjectPtr;
  m_v.ob = (acpObject*)pcObject;
}



/////////////////////////////////////////////////////////////////////

acpValue::~acpValue()
{
  empty();

} // acpValue destructor



/////////////////////////////////////////////////////////////////////

void acpValue::set(const float fValue)
{
  empty();

  m_eType = kFloat;
  m_v.f = fValue; 
}



/////////////////////////////////////////////////////////////////////

void acpValue::set(const int nValue)
{
  empty();

  m_eType = kInt;
  m_v.i = nValue; 
}



/////////////////////////////////////////////////////////////////////

void acpValue::set(const char* pString)
{
  if ((m_eType == kString) && (m_v.s == pString))
    return;

  empty();

  m_eType = kString;
  unsigned int len = aStringLen(pString);
  m_v.s = (char*)aMemAlloc((len + 1) * sizeof(char));
  aStringCopySafe(m_v.s, len + 1, pString);
}



/////////////////////////////////////////////////////////////////////

void acpValue::set(const bool bVal)
{
  empty();

  m_eType = kBoolean;
  m_v.b = bVal;
}


/////////////////////////////////////////////////////////////////////

void acpValue::set(
  const void* vp
)
{
  empty();

  m_eType = kVoidPtr;
  m_v.v = vp;
}


/////////////////////////////////////////////////////////////////////

void acpValue::set(
  const acpCallback* pcCallback
)
{
  empty();

  m_eType = kCallbackPtr;
  m_v.cb = (acpCallback*)pcCallback;
}


/////////////////////////////////////////////////////////////////////

void acpValue::set(
  const acpObject* pcObject
)
{
  empty();

  m_eType = kObjectPtr;
  m_v.ob = (acpObject*)pcObject;
}


/////////////////////////////////////////////////////////////////////

void acpValue::set(
  const acpValue* pValue
)
{
  if (this == pValue)
    return;

  switch (pValue->m_eType) {

  case kInt:
    set(pValue->m_v.i);
    break;

  case kFloat:
    set(pValue->m_v.f);
    break;

  case kString:
    set(pValue->m_v.s);
    break;

  case kBoolean:
    set(pValue->m_v.b);
    break;

  case kVoidPtr:
    set(pValue->m_v.v);
    break;

  case kCallbackPtr:
    set(pValue->m_v.cb);
    break;

  case kObjectPtr:
    set(pValue->m_v.ob);
    break;

  case kEmpty:
    empty();
    m_eType = kEmpty;
    break;
  }
}


/////////////////////////////////////////////////////////////////////
    
float acpValue::getFloatVal() const
{ 
  switch (m_eType) {
  	
  case kInt:
    return (float)m_v.i;
    break;
  
  case kBoolean:
    return (float)m_v.b;
    break;

  case kFloat:
    return m_v.f;
    break;
    
  default: // don't do anything for types we can't convert
    break;

  } // switch

  return 0.0f;
}


/////////////////////////////////////////////////////////////////////

const char* acpValue::getStringVal() const
{ 
  if (m_eType == kString)
    return m_v.s;

  return("");
}


/////////////////////////////////////////////////////////////////////

void acpValue::empty()
{
  switch (m_eType) {

  case kString:
    if (m_v.s)
      aMemFree(m_v.s);
    m_v.s = (char*)NULL;
    break;

  default:
    // do nothing unless additional resources are allocated
    break;
  }

  m_eType = kEmpty;

} // empty method

