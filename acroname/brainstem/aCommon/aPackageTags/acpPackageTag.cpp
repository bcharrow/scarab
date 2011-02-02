/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPackageTag.cpp                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of virtual robot data package       //
//		tag structure routines.				   //
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

#include "acpShort.h"
#include "acpInt32.h"
#include "acpCounterStream.h"
#include "acpException.h"
#include "acpPackageTag.h"


/////////////////////////////////////////////////////////////////////

const char* acpPackageTag::getTagTypeName() const
{
  // these names must be synchronized with the tag ID's in
  // acpPackageTagID.h
  static const char* names[] = {
    "views",
    "triangles",
    "sphere",
    "plugin",
    "dynamic",
    "patchbay",
    "plane",
    "box",
    "cylinder",
    "capsule",
    "motor",
    "instance",
    "property",
    "set",
    "get",
    "primitive",
    "template",
    "user-object",
    "initializer",
    "api-package",
    "link",
    "motorlink",
    "license",
    "simulation",
    "servo",
    "points"
  };
  
  if ((m_tagType <= 0) || (m_tagType > (aShort)sizeof(names)))
    throw acpException(aErrRange, "tag ID out of range");
    
  return names[m_tagType - 1];
}


/////////////////////////////////////////////////////////////////////

aInt32 acpPackageTag::writeToStream(
  aStreamRef stream) const
{
  // each tag starts with a numeric identifier (2-byte short)
  acpShort type(getTagType());
  type.writeToStream(stream);

  // the number of bytes in the tag should preceed the data
  acpCounterStream counter(aStreamLibRef(stream));
  writeData(counter);
  acpInt32 byteCount(counter.putCount());
  byteCount.writeToStream(stream);

  // followed by the tag's specific data
  writeData(stream);
  
  return byteCount;
}
