/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpASCIIHexFilter.cpp			                   //
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
#include "acpASCIIHexFilter.h"
#include "aStream.h"


char acpASCIIHexFilter::hex[16] = {
  '0', '1', '2', '3', 
  '4', '5', '6', '7', 
  '8', '9', 'A', 'B', 
  'C', 'D', 'E', 'F'
};

/////////////////////////////////////////////////////////////////////

acpASCIIHexFilter::acpASCIIHexFilter(
  aIOLib ioRef,
  aStreamRef passthrough, // = NULL
  const int nMaxLine // = 0
) :
  m_ioRef(ioRef),
  m_stream(NULL),
  m_passthrough(passthrough),
  m_nMaxLine(nMaxLine),
  m_nWidth(0)
{
  aErr err;
  aStream_Create(m_ioRef, getProc, putProc, deleteProc, 
  		 this, &m_stream, &err);
}


/////////////////////////////////////////////////////////////////////

acpASCIIHexFilter::~acpASCIIHexFilter()
{
  aErr err;
  aStreamRef stream = m_stream;
  m_stream = NULL;
  if (stream)
    aStream_Destroy(m_ioRef, stream, &err);
}


/////////////////////////////////////////////////////////////////////

aErr acpASCIIHexFilter::getProc(
  char *pData,
  void* ref
)
{
  acpASCIIHexFilter* pCS = (acpASCIIHexFilter*)ref;
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

aErr acpASCIIHexFilter::putProc(
  char *pData,
  void* ref
)
{
  acpASCIIHexFilter* pCS = (acpASCIIHexFilter*)ref;
  if (pCS->m_passthrough) {
    aStream* pStream = (aStream*)pCS->m_passthrough;
    if (pStream->putProc) {
      char c;
      c = hex[((unsigned char)*pData) / 16];
      pStream->putProc(&c, pStream->procRef);
      c = hex[((unsigned char)*pData) % 16];
      pStream->putProc(&c, pStream->procRef);
      pCS->m_nWidth += 2;
      if (pCS->m_nMaxLine && (pCS->m_nWidth >= pCS->m_nMaxLine)) {
        aStream_WriteLine(aStreamLibRef(pCS->m_passthrough),
        		  pCS->m_passthrough,
        		  "", NULL);
        pCS->m_nWidth = 0;
      }
      return aErrNone;
    } else
      return aErrMode;
  } else
    return aErrNone;
}


/////////////////////////////////////////////////////////////////////

aErr acpASCIIHexFilter::deleteProc(
  void* ref
)
{
  acpASCIIHexFilter* pCS = (acpASCIIHexFilter*)ref;
  if (pCS->m_stream) {
    pCS->m_stream = NULL;
    delete pCS;
  }
  return aErrNone;
}
