/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: acpException.cpp      	  		                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Base exception class for Acroname C++ code.        */
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

#include "aErr.h"
#include "aOSDefs.h"
#include "acpException.h"
#include "acpString.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

acpException::acpException(const acpException& e) :
  m_err(e.m_err),
  m_pMsg(NULL)
{
  size_t len = aStringLen(e.m_pMsg);
  m_pMsg = (char*)aMemAlloc((len + 1) * sizeof(char));
  if (m_pMsg)
    aStringCopySafe(m_pMsg, len + 1, e.m_pMsg);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

acpException::acpException(const aErr err,
			   const char* text) :			   
  m_err(err),
  m_pMsg(0L)
{
  size_t len = aStringLen(text);
  m_pMsg = (char*)aMemAlloc((len + 1) * sizeof(char));
  if (m_pMsg)
    aStringCopySafe(m_pMsg, len + 1, text);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

acpException::~acpException(void) 
{ 
  if (m_pMsg) 
    aMemFree(m_pMsg);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

const acpException&
acpException::operator= (acpException& e)
{
  if (m_pMsg)
    aMemFree(m_pMsg);

  m_err = e.m_err;
  size_t len = aStringLen(e.m_pMsg);
  m_pMsg = (char*)aMemAlloc((len + 1) * sizeof(char));
  if (m_pMsg)
    aStringCopySafe(m_pMsg, len + 1, e.m_pMsg);
  
  return *this;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

acpException::operator const char*(void) const
{ 
  return m_pMsg; 
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

const aErr 
acpException::error(void) const
{ 
  return m_err;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

const char* 
acpException::msg(void) const
{ 
  return m_pMsg; 
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifdef aTEST

#include "aMemLeakDebug.h"

// compile on Unix with:
//  g++ -DaUNIX -DaTEST -DaDEBUG -Wall aOSDefs.c aMemLeakDebug.c acpException.cpp

int main(const int argc, const char* argv[])
{
  printf("testing acpException\n");
  unsigned int errors = 0;

  try {
    throw acpException(aErrUnknown, "blah");
  } catch (const acpException& exception) {
    if (aStringCompare("blah", exception)
	|| (exception.error() != aErrUnknown)) {
      printf("exception failed: %s, %d\n", (const char*)exception,
	     exception.error());
      errors++;
    }
  }
  /* * * * * * * * * * * * * * * * * * * * */
  try {
    throw acpException(aErrBusy, "foobar");
  } catch (const acpException& exception) {
    if (aStringCompare("foobar", exception) 
	|| (exception.error() != aErrBusy)) {
      printf("exception failed: %s, %d\n", (const char*)exception,
	     exception.error());
      errors++;
    }
  }
  
  aLeakCheckCleanup();

  printf("  %d errors\n", errors);

  return errors;
}
#endif /* aTEST */
