/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: acpFloat.cpp 		  		                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: command line version of the compiler.              */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2008. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "aUtil.h"
#include "acpException.h"
#include "acpFloat.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * acpFloat constructor
 */

acpFloat::acpFloat(
  const aStreamRef stream
)
{
  aErr err = aErrNone;

  char buf[sizeof(aFloat)];

  if (aStream_Read(aStreamLibRef(stream), stream, buf, sizeof(aFloat), &err))
    throw acpException(err, "acpFloat: unable to read from stream");

  m_val = aUtil_RetrieveFloat(buf);
  
} // acpFloat constructor


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * acpFloat writeToStream method
 */

void acpFloat::writeToStream(
  aStreamRef stream
) const
{
  char buf[sizeof(aFloat)];

  aUtil_StoreFloat(buf, m_val);
  
  aErr err = aErrNone;
  if (aStream_Write(aStreamLibRef(stream), stream, buf, sizeof(aFloat), &err))
    throw acpException(err, "writing int32");

} // acpFloat writeToStream method
