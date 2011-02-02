/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPDFObject.cpp                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: PDF Object tag class.                              //
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
#include "acpPDFObject.h"
#include "acpGDPDF.h"

/////////////////////////////////////////////////////////////////////

acpPDFObject::acpPDFObject(
  acpGDPDF* pDoc,
  acpPDFObject* pParent,
  const aPDFObjectType eType // = kPDFTag
) :
  m_ioRef(pDoc->m_ioRef),
  m_nIndex(pDoc->m_objects.length() + 1),
  m_pParent(pParent),
  m_dictionaryBuffer(NULL),
  m_streamBuffer(NULL),
  m_eType(eType),
  m_nLines(0),
  m_pContent(NULL),
  m_bSubObject(false)
{
  aErr err;
  aAssert(m_ioRef);
  aStreamBuffer_Create(m_ioRef, 100, &m_dictionaryBuffer, &err);
  aAssert(err == aErrNone);
  aStreamBuffer_Create(m_ioRef, 100, &m_streamBuffer, &err);
  aAssert(err == aErrNone);

  aAssert(pDoc);
  pDoc->m_objects.add(this);
}


/////////////////////////////////////////////////////////////////////

acpPDFObject::acpPDFObject(
  const char* pType,
  aIOLib ioRef) :
  m_ioRef(ioRef),
  m_pParent(NULL),
  m_eType(kPDFTag),
  m_type(pType),
  m_bSubObject(true)
{
  aErr err;
  aAssert(m_ioRef);
  aStreamBuffer_Create(m_ioRef, 100, &m_dictionaryBuffer, &err);
  aAssert(err == aErrNone);
  aStreamBuffer_Create(m_ioRef, 100, &m_streamBuffer, &err);
  aAssert(err == aErrNone);
}


/////////////////////////////////////////////////////////////////////

acpPDFObject::~acpPDFObject()
{
  aErr err;
  if (m_dictionaryBuffer) {
    aStream_Destroy(m_ioRef, m_dictionaryBuffer, &err);
    aAssert(err == aErrNone);
    m_dictionaryBuffer = NULL;
  }
  if (m_streamBuffer) {
    aStream_Destroy(m_ioRef, m_streamBuffer, &err);
    aAssert(err == aErrNone);
    m_streamBuffer = NULL;
  }
}


/////////////////////////////////////////////////////////////////////

void acpPDFObject::addDictionary(
  const char* text
)
{
  aErr err;
  unsigned int len = aStringLen(text);
  aStream_Write(m_ioRef, m_dictionaryBuffer, text, len, &err);
  aAssert(err == aErrNone);
}


/////////////////////////////////////////////////////////////////////

void acpPDFObject::addDictionaryLine(
  const char* line
)
{
  aErr err;
  aStream_WriteLine(m_ioRef, m_dictionaryBuffer, line, &err);
  aAssert(err == aErrNone);  
  m_nLines++;
}


/////////////////////////////////////////////////////////////////////

void acpPDFObject::addStream(
  const char* text
)
{
  aErr err;
  unsigned int len = aStringLen(text);
  aStream_Write(m_ioRef, m_streamBuffer, text, len, &err);
  aAssert(err == aErrNone);  
}


/////////////////////////////////////////////////////////////////////

void acpPDFObject::addStreamLine(
  const char* line
)
{
  aErr err;
  aStream_WriteLine(m_ioRef, m_streamBuffer, line, &err);
  aAssert(err == aErrNone);
  m_nLines++;
}


/////////////////////////////////////////////////////////////////////

void acpPDFObject::addDictionaryObject(
  acpPDFObject* pSubObject) 
{
  m_subObjects.add(pSubObject);
}


/////////////////////////////////////////////////////////////////////

void acpPDFObject::write(
  aStreamRef output)
{
  aErr err;
  acpString tagLine;

  if (!m_bSubObject) {
    tagLine = m_nIndex;
    tagLine += " 0 obj";
    aStream_WriteLine(m_ioRef, output, tagLine, &err);
    aAssert(err == aErrNone);
  } else {
    tagLine = (char*)m_type;
    tagLine += ' ';
    aStream_Write(m_ioRef, output, tagLine, tagLine.length(), &err);
    aAssert(err == aErrNone);
  }

  // see if there was any stream data
  aMemSize streamSize;
  aStreamBuffer_Get(m_ioRef, m_streamBuffer, &streamSize, 
		    (char**)NULL, &err);
  aAssert(err == aErrNone);

  // if there was stream data, add the length info
  if (streamSize) {
    tagLine.format("/Length %d", (int)streamSize);
    addDictionaryLine(tagLine);
  }

  switch (m_eType) {
  case kPDFList:
    aStream_Write(m_ioRef, output, "[ ", 2, &err);
    break;
  case kPDFTag:
    aStream_WriteLine(m_ioRef, output, "<<", &err);
    break;
  default:
    break;
  } // switch
  aAssert(err == aErrNone);

  // write the dictionary contents
  aStream_Flush(m_ioRef, m_dictionaryBuffer, output, &err);
  aAssert(err == aErrNone);

  // now, dump any subobjects
  aLISTITERATE(acpPDFObject, m_subObjects, pObject) {
    pObject->write(output);
  }

  switch (m_eType) {
  case kPDFList:
    aStream_WriteLine(m_ioRef, output, " ]", &err);
    break;
  case kPDFTag:
    aStream_WriteLine(m_ioRef, output, ">>", &err);
    break;
  default:
    break;
  } // switch
  aAssert(err == aErrNone);

  // now, write the stream data if present
  if (streamSize) {
    aStream_WriteLine(m_ioRef, output, "stream", &err);
    aAssert(err == aErrNone);

    aStream_Flush(m_ioRef, m_streamBuffer, output, &err);
    aAssert(err == aErrNone);

    aStream_WriteLine(m_ioRef, output, "endstream", &err);
    aAssert(err == aErrNone);
  }

  if (!m_bSubObject) {	
    aStream_WriteLine(m_ioRef, output, "endobj", &err);
    aAssert(err == aErrNone);
  }
}
