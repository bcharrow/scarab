/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTextBuffer.cpp                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of a text buffer object.            //
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
#include "acpException.h"
#include "acpTextBuffer.h"


/////////////////////////////////////////////////////////////////////

acpTextBuffer::acpTextBuffer (
  aIOLib ioRef
) :
  m_ioRef(ioRef)
{
  aErr err;
  aStreamBuffer_Create(m_ioRef, 32, &m_buffer, &err);

  aAssert(err == aErrNone);
}

/////////////////////////////////////////////////////////////////////



acpTextBuffer::~acpTextBuffer()
{
  aErr err;
  aStream_Destroy(m_ioRef, m_buffer, &err);
  aAssert(err == aErrNone);
}



/////////////////////////////////////////////////////////////////////

void acpTextBuffer::add(const char* pText)
{
  aErr err;
  aStream_Write(m_ioRef, m_buffer, pText, aStringLen(pText), &err);
  aAssert(err == aErrNone);
}



/////////////////////////////////////////////////////////////////////

void acpTextBuffer::add(const float fValue)
{
  char floatString[30];
  aString_FormatFloat(fValue, floatString);
  add(floatString);
}



/////////////////////////////////////////////////////////////////////

void acpTextBuffer::add(const int nValue)
{
  char num[10];
  aStringFromInt(num, nValue);
  add(num);
}



/////////////////////////////////////////////////////////////////////

const char* acpTextBuffer::getBuffer()
{
  aAssert(m_buffer);
  char* pValue;
  aErr err;

  aStream_Write(m_ioRef, m_buffer, "\0", 1, &err);
  aAssert(err == aErrNone);

  aStreamBuffer_Get(m_ioRef, m_buffer, NULL, &pValue, &err);
  aAssert(err == aErrNone);
    
  return pValue;
}
