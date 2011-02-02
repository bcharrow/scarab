/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpStringIO.cpp                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the acpString IO utility object. //
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

#include "acpStringIO.h"
#include "acpException.h"

/////////////////////////////////////////////////////////////////////

acpStringIO::acpStringIO(const aStreamRef stream)
{
  aIOLib ioRef = aStreamLibRef(stream);
  aStreamRef buffer;
  aErr err;
  static const char* pErr = "creating string from stream";
  
  if (aStreamBuffer_Create(ioRef, 32, &buffer, &err))
    throw acpException(err, pErr);
  
  char b;
  do {
    if (aStream_Read(ioRef, stream, &b, 1, &err))
      throw acpException(err, pErr);
    
    if (aStream_Write(ioRef, buffer, &b, 1, &err))
      throw acpException(err, pErr);
  } while (b != 0);

  aMemSize size;
  char* pString;
  if (aStreamBuffer_Get(ioRef, buffer, &size, &pString, &err))
    throw acpException(err, pErr);
  
  // ensure we have enough space to store the new string
  if (m_nStorageCapacity < size) {
    aMemFree(m_pStorage);
    m_pStorage = (char*)aMemAlloc(size);
    if (!m_pStorage)
      throw acpException(aErrMemory, "allocating IO string");
  }

  aStringCopySafe(m_pStorage, m_nStorageCapacity, pString);
  
  if (aStream_Destroy(ioRef, buffer, &err))
    throw acpException(err, pErr);
}


/////////////////////////////////////////////////////////////////////
// acpString writeToStream method

aErr acpStringIO::writeToStream(aStreamRef stream) const
{
  aMemSize size = (aMemSize)aStringLen(m_pStorage);
  
  aErr err = aErrNone;
  
  aStream_Write(aStreamLibRef(stream), stream,
  		(char*)m_pStorage, size, &err);
  
  char terminator = '\0';
  aStream_Write(aStreamLibRef(stream), stream,
  		(char*)&terminator, 1, &err);
  
  return err;
}


/////////////////////////////////////////////////////////////////////
// acpString getStream method

aStreamRef acpStringIO::getStream(aIOLib ioRef)
{
  aErr ioErr;
  aStreamRef streamRef;
  
  m_current = 0;

  if (aStream_Create(ioRef, sStreamGet,
  		     NULL, NULL,
  		     this, &streamRef, &ioErr))
    throw acpException(aErrIO, "creating string stream");
  
  return streamRef;
}


/////////////////////////////////////////////////////////////////////
// acpString streamGet static method

aErr 
acpStringIO::sStreamGet(char* pData,
			void* ref)
{
  if (!pData)
    return aErrParam;

  acpStringIO* pString = (acpStringIO*)ref;
  
  if (pString->m_current >= pString->length())
    return aErrEOF;
  
  *pData = pString->m_pStorage[pString->m_current++];

  return aErrNone;
}
