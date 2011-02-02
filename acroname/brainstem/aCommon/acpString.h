/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpString.h                                               //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the acpString utility object.        //
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

#ifndef _acpString_H_
#define _acpString_H_

#include <stdarg.h>
#include "aUtil.h"

class aUTIL_EXPORT acpString {
  public:
                                acpString(void);
                                acpString(const acpString& string);
                                acpString(const char* pValue);
                                acpString(const int nValue);
    virtual                     ~acpString(void);

                                operator const char*(void) const; 
				operator char*(void) const; 

    const acpString& 		operator=(const acpString& RHS);
    const acpString& 		operator=(const char* pRHS);
    const acpString&		operator=(char* pRHS);

    const char*		        operator+=(const acpString& RHS);
    const char*			operator+=(const char* pRHS);
    const char* 		operator+=(const char cRHS);
    const char* 		operator+=(const unsigned char cRHS);
    const char* 		operator+=(const short sRHS);
    const char* 		operator+=(const unsigned short usRHS);
    const char* 		operator+=(const int iRHS);
    const char* 		operator+=(const unsigned int uiRHS);
    const char* 		operator+=(const long lRHS);
    const char* 		operator+=(const unsigned long ulRHS);
    const char* 		operator+=(const float fRHS);

    const bool                  operator==(const acpString& rhs) const;
    const bool                  operator==(char* rhs) const;
    const bool                  operator==(const char* rhs) const;

    const bool                  operator!=(const acpString& rhs) const;
    const bool                  operator!=(char* rhs) const;
    const bool                  operator!=(const char* rhs) const;

    const size_t	 	length(void) const;
    const char*                 trim(void);
    const char*                 truncate(const char* pSuffix);
    const char*                 substring(const size_t offset,
					  const size_t length);
    void			lowercase(void);
    const char*                 capitalize(void);
    const bool                  endsWith(const char* suffix) const;
    const char*                 format(const char* fmt, ...);
    void                        copyToBuffer(char* buffer, 
					     const size_t max) const;

  protected:
    char*			m_pStorage;
    size_t	                m_nStorageCapacity;
    size_t                      m_length;

  private:
    void                        assume(acpString& s);
};



/////////////////////////////////////////////////////////////////////
// inline functions (must be in header file)
inline acpString::operator const char*(void) const 
  { return m_pStorage; }
inline acpString::operator char*(void) const 
  { return m_pStorage; }
inline const size_t acpString::length(void) const
  { return m_length; }
inline const bool acpString::operator==(const acpString& rhs) const
  { return !aStringCompare(m_pStorage, rhs.m_pStorage); }
inline const bool acpString::operator==(char* rhs) const
  { return !aStringCompare(m_pStorage, rhs); }
inline const bool acpString::operator==(const char* rhs) const
  { return !aStringCompare(m_pStorage, rhs); }
inline const bool acpString::operator!=(const acpString& rhs) const
  { return (aStringCompare(m_pStorage, rhs.m_pStorage) != 0); }
inline const bool acpString::operator!=(char* rhs) const
  { return (aStringCompare(m_pStorage, rhs) != 0); }
inline const bool acpString::operator!=(const char* rhs) const
  { return (aStringCompare(m_pStorage, rhs) != 0); }

#endif // _acpString_H_
