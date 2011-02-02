/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpInt32.cpp 		 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of virtual robot data package       //
//		routines.					   //
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
#include "acpInt32.h"
#include "acpException.h"


/////////////////////////////////////////////////////////////////////

acpInt32::acpInt32(
  aStreamRef stream
)
{
  aErr err;
  char buf[sizeof(aInt32)];
  if (aStream_Read(aStreamLibRef(stream), stream,
  		   buf, sizeof(aInt32), &err))
    throw acpException(err, "reading int32");

  m_val = aUtil_RetrieveInt(buf);
}


/////////////////////////////////////////////////////////////////////

void acpInt32::writeToStream(
  aStreamRef stream
) const
{
  char buf[sizeof(aInt32)];

  aUtil_StoreInt(buf, m_val);

  aErr err;
  if (aStream_Write(aStreamLibRef(stream), stream, buf, sizeof(aInt32), &err))
    throw acpException(err, "writing int32");
}
