/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPackage.cpp	 	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of acroname package.	           //
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

#include "aIO.h"
#include "aUtil.h"
#include "aVersion.h"

#include "acpByte.h"
#include "acpShort.h"
#include "acpInt32.h"
#include "acpException.h"
#include "acpPackage.h"
#include "acpCounterStream.h"


/////////////////////////////////////////////////////////////////////

// must be 4 characters
const char* acpPackage::m_signature = "ACSP";


/////////////////////////////////////////////////////////////////////

acpPackage::acpPackage(
  acpPackageProgress* pProgress,
  aIOLib ioRef,
  aByte versionMajor,
  aByte versionMinor,
  aInt32 build,
  aShort fileRev,
  aStreamRef stream // = NULL
) :
  m_progressIndicator(pProgress),
  m_initStream(stream),
  m_versionMajor(versionMajor),
  m_versionMinor(versionMinor),
  m_build(build),
  m_fileRev(fileRev),
  m_ioRef(ioRef)
{
}


/////////////////////////////////////////////////////////////////////

acpPackage::~acpPackage()
{
}


/////////////////////////////////////////////////////////////////////

void acpPackage::initialize()
{
  if (m_initStream) {
    aErr err;
    char signature[4];
    if (aStream_Read(m_ioRef, m_initStream, signature, 4, &err)
        || (signature[0] != m_signature[0])
        || (signature[1] != m_signature[1])
        || (signature[2] != m_signature[2])
        || (signature[3] != m_signature[3])
       )
      throw acpException(err, "bad file format");

    // get the version bytes
    acpByte versionMajor(m_initStream);
    acpByte versionMinor(m_initStream);

    // get and verify the build
    acpInt32 build(m_initStream);
    m_build = build;

    acpShort fileRev(m_initStream);
    if (fileRev != m_fileRev) {
      acpString msg("Incompatible file version (got ");
      msg += (long)fileRev;
      msg += " expecting ";
      msg += (long)m_fileRev;
      msg += "), needs repackaging";
      throw acpException(err, (char*)msg);
    }

    if ((m_versionMajor != versionMajor) || (m_versionMinor != versionMinor))
      throw acpException(err, "incompatible file version, needs repackaging");

    // the rest of the file is compressed using zlib
    aStreamRef zlibStream;
    if (aStream_CreateZLibFilter(m_ioRef, m_initStream, 
			         aFileModeReadOnly,
  	  		         &zlibStream, &err))
      throw acpException(err, "creating decompression stream");

    // read in each tag
    acpInt32 nTags(zlibStream);
    for (int i = 0; i < nTags; i++) {
      acpShort ID(zlibStream);

      acpInt32 tagByteCount(zlibStream);
      aAssert(tagByteCount != 0);

      if (m_progressIndicator)
      	m_progressIndicator->updateProgress(nTags, i);

      acpPackageTag* pTag;
      acpListIterator<acpPackageTag> tags(m_recognizedTags);

      acpPackageTag* pNewTag = NULL;
      while ((pTag = tags.next())) {
        if (pTag->getTagType() == ID) {
      	  pNewTag = pTag->factoryFromStream(zlibStream);
      	  pNewTag->setID((aShort)(i+1));
          m_tags.add(pNewTag);
          break;
        }
      }

      // tag was not found, was it added to the tag list?
      aAssert(pNewTag);

    } // for

    if (aStream_Destroy(m_ioRef, zlibStream, &err))
      throw acpException(err, "destroying compression stream"); 
  } // if m_initStream
}


/////////////////////////////////////////////////////////////////////

aInt32 acpPackage::getTagChild(
  const aInt32 nTagID
) const
{
  aLISTITERATE(acpPackageTag, m_tags, pTag) {
    if (pTag->getOwnerID() == nTagID)
      return pTag->getID();
  }

  // error! child sought and none found
  aAssert(0);
  return -1;
}


/////////////////////////////////////////////////////////////////////

void acpPackage::addTag(
  acpPackageTag* pTag
) 
{
  m_tags.add(pTag);
  pTag->setID((short)m_tags.length());
}


/////////////////////////////////////////////////////////////////////

void acpPackage::writeToStream(
  aStreamRef stream
)
{
  aErr err;

  if (aStream_Write(m_ioRef, stream,
  		    m_signature, 4, &err))
    throw acpException(err, "writing file signature");
  
  // add the version bytes
  acpByte high(m_versionMajor);
  high.writeToStream(stream);
  acpByte low(m_versionMinor);
  low.writeToStream(stream);

  // and the build
  acpInt32 build(m_build);
  build.writeToStream(stream);

  // and file version
  acpShort fileVersion(m_fileRev);
  fileVersion.writeToStream(stream);

  // the rest of the file is compressed using zlib
  aStreamRef zlibStream;
  if (aStream_CreateZLibFilter(m_ioRef, stream, 
  			       aFileModeWriteOnly,
  			       &zlibStream, &err))
    throw acpException(err, "creating compression stream");

  acpInt32 tagCount(m_tags.length());
  tagCount.writeToStream(zlibStream);

#ifdef aDEBUG
  int i = 1;
#endif
  aLISTITERATE(acpPackageTag, m_tags, pTag) {
#ifdef aDEBUG
    printf("writing tag %d of type 0x%.4X(%s) with owner %d\n", 
           i++, 
           pTag->getTagType(), 
           (char*)pTag->getTagTypeName(), 
           (int)pTag->getOwnerID());
#endif // aDEBUG
    pTag->writeToStream(zlibStream);
  }

  if (aStream_Destroy(m_ioRef, zlibStream, &err))
    throw acpException(err, "destroying compression stream");

} // acpPackage writeToStream method


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpPackage::findTagByType(
  const aShort nType) 
{
  aLISTITERATE(acpPackageTag, m_tags, pTag) {
    if (pTag->getTagType() == nType)
      return pTag;
  }

  return NULL;
}
