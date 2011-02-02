/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpCounterStream.cpp			                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Utility class for counting stream I/O.		   //
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

#include "acpException.h"
#include "acpCounterStream.h"
#include "aStream.h"


/////////////////////////////////////////////////////////////////////

acpCounterStream::acpCounterStream(
  aIOLib ioRef,
  aStreamRef passthrough // = NULL
) :
  m_ioRef(ioRef),
  m_passthrough(passthrough),
  m_nGets(0),
  m_nPuts(0)
{
  aErr err;
  aStream_Create(m_ioRef, getProc, putProc, deleteProc, 
  		 this, &m_stream, &err);
}


/////////////////////////////////////////////////////////////////////

acpCounterStream::~acpCounterStream()
{
  aErr err;
  aStreamRef stream = m_stream;
  m_stream = NULL;
  if (stream)
    aStream_Destroy(m_ioRef, stream, &err);
}


/////////////////////////////////////////////////////////////////////

aErr acpCounterStream::getProc(
  char *pData,
  void* ref
)
{
  acpCounterStream* pCS = (acpCounterStream*)ref;
  pCS->m_nGets++;
  if (pCS->m_passthrough) {
    aStream* pStream = (aStream*)pCS->m_passthrough;
    if (pStream->getProc)
      return pStream->getProc(pData, pStream->procRef);
    else
      return aErrMode;
  } else
    return aErrNone;
}


/////////////////////////////////////////////////////////////////////

aErr acpCounterStream::putProc(
  char *pData,
  void* ref
)
{
  acpCounterStream* pCS = (acpCounterStream*)ref;
  pCS->m_nPuts++;
  if (pCS->m_passthrough) {
    aStream* pStream = (aStream*)pCS->m_passthrough;
    if (pStream->putProc)
      return pStream->putProc(pData, pStream->procRef);
    else
      return aErrMode;
  } else
    return aErrNone;
}


/////////////////////////////////////////////////////////////////////

aErr acpCounterStream::deleteProc(
  void* ref
)
{
  acpCounterStream* pCS = (acpCounterStream*)ref;
  if (pCS->m_stream) {
    pCS->m_stream = NULL;
    delete pCS;
  }
  return aErrNone;
}
