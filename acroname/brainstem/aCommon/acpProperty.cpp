/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpProperty.cpp                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API property object.  //
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

#include "aUtil.h"
#include "acpProperty.h"
#include "acpObject.h"


/////////////////////////////////////////////////////////////////////

acpProperty::acpProperty (
  const char* pName,
  const aPROPERTY_FLAGS ucType,
  const char* pDescription // = NULL
) :
  m_typeFlags(ucType),
  m_pDescription(NULL)
{
  aAssert(pName);

  unsigned int len = (unsigned int)(aStringLen(pName) + 1);
  m_pName = (char*)aMemAlloc(len);
  if (m_pName) {
    aStringCopy(m_pName, pName);
  }

  if (pDescription) {	
    len = (unsigned int)(aStringLen(pDescription) + 1);
    m_pDescription = (char*)aMemAlloc(len);
    if (m_pDescription)
      aStringCopy(m_pDescription, pDescription);
  }

} // acpProperty constructor


/////////////////////////////////////////////////////////////////////

acpProperty::~acpProperty()
{
  if (m_pDescription)
    aMemFree(m_pDescription);

  if (m_pName)
    aMemFree(m_pName);

} // acpProperty destructor


/////////////////////////////////////////////////////////////////////

aErr acpProperty::readValue(aStreamRef stream,
			    acpValue* pValue)
{
  aErr err = aErrNone;
  aIOLib ioRef = aStreamLibRef(stream);
  aPROPERTY_FLAGS type = m_typeFlags & aPROPERTY_TYPE_MASK;

  switch (type) {
  
  case aPROPERTY_FLAG_FLOAT:
    char buf[sizeof(float)];
    if (!aStream_Read(ioRef, stream, 
    		      buf, sizeof(float), &err))
      pValue->set(aUtil_RetrieveFloat(buf));
    break;

  case aPROPERTY_FLAG_INT:
    int i;
    if (!aStream_Read(ioRef, stream, 
    		     (char*)&i, sizeof(i), &err))
      pValue->set(aUtil_RetrieveInt((char*)&i));
    break;

  case aPROPERTY_FLAG_STRING:
    aStreamRef inputBuffer;
    if (!aStreamBuffer_Create(ioRef, 100, 
    			      &inputBuffer, &err)) {
      char c = 1;
      while (!aStream_Read(ioRef, stream, &c, 1, &err)) {
        aStream_Write(ioRef, inputBuffer, &c, 1, &err);
        if (c == 0)
          break;
      }
      char* pString;
      aStreamBuffer_Get(ioRef, inputBuffer, 
      			NULL, &pString, &err);
      pValue->set(pString);
      aStream_Destroy(ioRef, inputBuffer, &err);
    }
    break;
  
  default:
    // not implemented
    aAssert(0);
    break;
    
  } // switch
  
  return err;

} // acpProperty readValue method


/////////////////////////////////////////////////////////////////////

void acpProperty::setValue(
  const acpValue* pValue
)
{
} // acpProperty setValue method


/////////////////////////////////////////////////////////////////////

void acpProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue
)
{
  aAssert(pObject->m_pValues);
  acpValue* pLookupValue = (*(pObject->m_pValues))[m_nIndex];
  if (pValue && pLookupValue)
    pValue->set(pLookupValue);

} // acpProperty getValue method
