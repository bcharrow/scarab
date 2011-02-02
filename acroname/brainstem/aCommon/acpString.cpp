/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpString.cpp                                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the acpString utility object.    //
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
#include "acpString.h"


// Value must be 0 or greater, this extra padding allows for 
// concatenations without re-allocation (within the padding amount)
// This value is a typical memory/speed constraint.
#define STRINGPADDINGSIZE 32

/////////////////////////////////////////////////////////////////////
// utility defines
#define CONVERSIONBUFSIZE 20
#define ALLOC(len) {						    \
  m_pStorage = (char*)aMemAlloc(((len) + STRINGPADDINGSIZE + 1)	    \
				 * sizeof(char));		    \
  if (!m_pStorage)						    \
    throw acpException(aErrMemory, "allocating a string");	    \
  m_nStorageCapacity = (len) + STRINGPADDINGSIZE;	            \
}
#define APPEND_NUMBER_CONVERSION(s, arg, type) {                    \
  char buf[CONVERSIONBUFSIZE];					    \
  aSNPRINTF(buf, CONVERSIONBUFSIZE, type, arg);			    \
  size_t len = aStringLen(buf);				            \
  if ((s.m_length + len) > s.m_nStorageCapacity) {		    \
    char* pTemp = s.m_pStorage;					    \
    ALLOC(s.m_length + len);					    \
    aStringCopySafe(s.m_pStorage, s.m_nStorageCapacity + 1, pTemp); \
    aMemFree(pTemp);						    \
  }								    \
  aStringCopySafe(&s.m_pStorage[s.m_length],                        \
                  s.m_nStorageCapacity - s.m_length + 1, buf);      \
  s.m_length += len;						    \
}

/////////////////////////////////////////////////////////////////////
// acpString constructor

acpString::acpString(void):
  m_pStorage(NULL),
  m_nStorageCapacity(0),
  m_length(0)
{
  ALLOC(0);
  m_pStorage[0] = 0;
  m_length = 0;

} // acpString constructor


/////////////////////////////////////////////////////////////////////
// acpString constructor

acpString::acpString(const acpString& string) :
  m_pStorage(NULL),
  m_nStorageCapacity(0),
  m_length(0)
{
  size_t len = string.m_length;
  ALLOC(len);
  aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, 
		  string.m_pStorage);
  m_length = len;
}


/////////////////////////////////////////////////////////////////////
// acpString constructor

acpString::acpString(const char* pValue) :
  m_pStorage(NULL),
  m_nStorageCapacity(0),
  m_length(0)
{
  size_t len = aStringLen(pValue);
  ALLOC(len);
  aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pValue);
  m_length = (unsigned int)len;
  
} // acpString constructor


/////////////////////////////////////////////////////////////////////
// acpString constructor

acpString::acpString(
  const int nValue
) :
  m_pStorage(NULL),
  m_nStorageCapacity(0),
  m_length(0)
{
  char buf[CONVERSIONBUFSIZE];
  aSNPRINTF(buf, CONVERSIONBUFSIZE, "%d", nValue);
  size_t len = aStringLen(buf);
  ALLOC(len);
  aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, buf);
  m_length = len;
  
} // acpString constructor



/////////////////////////////////////////////////////////////////////
// acpString destructor

acpString::~acpString(void)
{ 
  if (m_pStorage) 
    aMemFree(m_pStorage);
#ifdef aDEBUG
  m_pStorage = NULL;
  m_nStorageCapacity = 0;
  m_length = 0;
#endif // aDEBUG
}


/////////////////////////////////////////////////////////////////////
// acpString operator=

const acpString& 
acpString::operator=(const acpString& RHS)
{
  if (RHS != *this) {
    size_t len = RHS.m_length;
    if (len > m_nStorageCapacity) {
      aMemFree(m_pStorage);
      ALLOC(len);
    }
    aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, 
	            RHS.m_pStorage);
    m_length = len;
  }
  
  return *this;
  
} // acpString = (acpString&) operator


/////////////////////////////////////////////////////////////////////
// acpString operator=

const acpString& 
acpString::operator=(const char* pRHS)
{
  if (pRHS) {
    size_t len = aStringLen(pRHS);
    if (m_nStorageCapacity < len) {
      aMemFree(m_pStorage);
      ALLOC(len);
    }
    aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pRHS);
    m_length = len;
  }

  return *this;
  
} // acpString = (const char*) operator


/////////////////////////////////////////////////////////////////////
// acpString operator=

const acpString& 
acpString::operator=(char* pRHS)
{
  if (pRHS) {
    size_t len = aStringLen(pRHS);
    if (m_nStorageCapacity < len) {
      aMemFree(m_pStorage);
      ALLOC(len);
    }
    aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pRHS);
    m_length = len;
  }
  
  return *this;
  
} // acpString = (char*) operator


/////////////////////////////////////////////////////////////////////
// acpString operator

const char* 
acpString::operator+=(const acpString& RHS)
{
  if (RHS == *this) {
    // weird case where we are concatenating ourselves which
    // creates an overlap
    char* pTemp = m_pStorage;
    size_t newLen = m_length * 2;
    ALLOC(newLen);
    aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pTemp);
    aStringCopySafe(&m_pStorage[m_length], 
		    m_nStorageCapacity - m_length + 1, pTemp);
    aMemFree(pTemp);
    m_length = newLen;
  } else {
    size_t len = RHS.m_length;
    if ((m_length + len) > m_nStorageCapacity) {
      char* pTemp = m_pStorage;
      ALLOC(m_length + len);
      aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pTemp);
      aMemFree(pTemp);
    }
    aStringCopySafe(&m_pStorage[m_length], 
		    m_nStorageCapacity - m_length + 1, 
		    RHS.m_pStorage);
    m_length += len;
  }
  
  return m_pStorage;
  
} // acpString += (acpString&) operator


/////////////////////////////////////////////////////////////////////
// acpString operator

const char* 
acpString::operator+=(const char* pRHS)
{
  if (pRHS) {
    size_t len = aStringLen(pRHS);
    if ((m_length + len) > m_nStorageCapacity) {
      char* pTemp = m_pStorage;
      ALLOC(m_length + len);
      aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pTemp);
      aMemFree(pTemp);
    }
    aStringCopySafe(&m_pStorage[m_length], 
		    m_nStorageCapacity - m_length + 1, pRHS);
    m_length += len;
  }

  return m_pStorage;

} // acpString += (char*) operator


/////////////////////////////////////////////////////////////////////
// acpString operator

const char* 
acpString::operator+=(const char cRHS)
{
  if ((m_length + 1) > m_nStorageCapacity) {
    char* pTemp = m_pStorage;
    ALLOC(m_length + 1);
    aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pTemp);
    aMemFree(pTemp);
  }
  m_pStorage[m_length++] = cRHS;
  m_pStorage[m_length] = 0;

  return m_pStorage;
  
} // acpString += (const char) operator


/////////////////////////////////////////////////////////////////////
// acpString operator

const char* 
acpString::operator+=(const unsigned char ucRHS)
{
  if ((m_length + 1) > m_nStorageCapacity) {
    char* pTemp = m_pStorage;
    ALLOC(m_length + 1);
    aStringCopySafe(m_pStorage, m_nStorageCapacity + 1, pTemp);
    aMemFree(pTemp);
  }
  m_pStorage[m_length++] = ucRHS;
  m_pStorage[m_length] = 0;
  
  return m_pStorage;
  
} // acpString += (const unsigned char) operator


/////////////////////////////////////////////////////////////////////
// acpString operator

const char*
acpString::operator+=(const short sRHS)
{
  APPEND_NUMBER_CONVERSION((*this), sRHS, "%hd"); 
  return m_pStorage;
}



/////////////////////////////////////////////////////////////////////
// acpString operator

const char*
acpString::operator+=(const unsigned short usRHS)
{
  APPEND_NUMBER_CONVERSION((*this), usRHS, "%hu"); 
  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString operator

const char*
acpString::operator+=(const int iRHS)
{
  APPEND_NUMBER_CONVERSION((*this), iRHS, "%d"); 
  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString operator

const char*
acpString::operator+=(const unsigned int uiRHS)
{
  APPEND_NUMBER_CONVERSION((*this), uiRHS, "%u");
  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString operator

const char* acpString::operator+=(const long lRHS)
{
  APPEND_NUMBER_CONVERSION((*this), lRHS, "%ld"); 
  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString operator

const char* acpString::operator+=(const unsigned long ulRHS)
{
  APPEND_NUMBER_CONVERSION((*this), ulRHS, "%lu"); 
  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString += float operator

const char* acpString::operator+=(const float fRHS)
{
  APPEND_NUMBER_CONVERSION((*this), fRHS, "%f");

  // remove the trailing zeros
  while ((m_length > 1) && (m_pStorage[m_length - 1] == '0')) {
    if (m_pStorage[m_length - 2] != '.') {
      m_pStorage[m_length - 1] = 0;
      m_length--;
    } else 
      break;
  }

  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString trim

const char* 
acpString::trim()
{
  if (m_length) {
    size_t w = 0;
    char* c = &m_pStorage[m_length - 1];

    // first, move backwards while there are trailing white chars
    while ((w < m_length) 
	   && ((*c == '\t') || (*c == ' ') 
	       || (*c == '\n') || (*c == '\r'))) {
      c--;
      w++;
    }

    // truncate if there were white characters
    if (w) {
      c++;
      *c = 0;
      m_length -= w;
    }

    // skip up to the first non-white character
    w = 0;
    c = m_pStorage;
    while (*c 
	   && ((*c == '\t') || (*c == ' ') 
	       || (*c == '\n') || (*c == '\r'))) {
      c++;
      w++;
    }

    // slide non-white characters down to the front of the string
    if (w) {
      size_t i;
      for(i = w; i < m_length; i++)
	m_pStorage[i - w] = m_pStorage[i];
      m_pStorage[i - w] = 0;
      m_length -= w;
    }
  }

  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString truncate

const char* 
acpString::truncate(const char* pSuffix) 
{
  // skip to the end of the suffix
  const char* s = pSuffix;
  while (*s) s++;
  
  // p and s are now at the end of the strings (null terminator)
  
  // match backwards till either suffix is done or string
  // is done
  do {
    if (m_pStorage[m_length] != *s) 
      break;
    if (s == pSuffix) {
      m_pStorage[m_length] = 0;
      break;
    }
    m_length--; 
    s--;

    if (s == pSuffix) {
      m_pStorage[m_length] = 0;
      break;
    }
    
  } while ((s != pSuffix) && (m_length));

  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// acpString substring

const char* 
acpString::substring(const size_t offset, 
		     const size_t len) 
{
  if (len == 0) {
    // case where we truncate to nothing
    m_length = 0;
  } else if (len > 0) {
    if (offset == 0) {
      // case where we just truncate the back side
      if (len < m_length) {
	m_length = len;
      }
    } else if (offset < m_length) {
      // case where we toss the front
      size_t end = offset + len;
      if (end > m_length)
	end = m_length;
      size_t i;
      for (i = 0; i < end - offset; i++)
	m_pStorage[i] = m_pStorage[offset + i];
      m_length = i;
    }
  }

  // make sure the result it null terminated
  m_pStorage[m_length] = 0;

  return m_pStorage;

} // substring


/////////////////////////////////////////////////////////////////////
// acpString lowercase method

void 
acpString::lowercase(void) 
{
  char* p = m_pStorage;
  while (p && *p) {
    if ((*p >= 'A') && (*p <= 'Z'))
      *p = (char)((int)*p - 'A' + 'a');
    p++;
  }
} // lowercase


/////////////////////////////////////////////////////////////////////
// capitalize

const char* 
acpString::capitalize(void)
{
  if (m_length > 0) {
    char* p = m_pStorage;;
    if ((*p >= 'a') && (*p <= 'z'))
      *p += 'A' - 'a';
  }
  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// endsWith method

const bool 
acpString::endsWith(const char* suffix) const
{
  if (suffix) {
    size_t len = aStringLen(suffix);
    if (len <= m_length) {
      size_t pos = m_length - len;
      return !aStringCompare(&m_pStorage[pos], suffix);
    }
  }
  return false;
}


/////////////////////////////////////////////////////////////////////
// format method

const char*
acpString::format(const char* fmt, ...)
{
  va_list va;
  va_start(va, fmt);

  const char* f = fmt;
  acpString result;
  bool bEscape = false;

  // now accumulate the remainder
  while (*f) {
    switch (*f) {
	
      case '%':
	if (!bEscape) {
	  f++;
	  
	  int zpad = 0;
	  
	  // see if we have a zero-padding specification
	  if (*f == '0') {
	    f++;
	    while ((*f >= '0') && (*f <= '9')) {
	      zpad *= 10;
	      zpad += *f - '0';
	      f++;
	    }
	  }
	  
	  acpString bit;
	  
	  switch (*f) {
	    case 's':
	      bit += va_arg(va, const char*);
	      break;
	    case 'd':
	      bit += va_arg(va, int);
	      break;
	    case 'u':
	      bit += va_arg(va, unsigned int);
	      break;
	    case 'c':
	    { int c = va_arg(va, int);	      
	      bit += (char)c;
	    } break;
	    case 'x': {
	      unsigned int x = va_arg(va, unsigned int);
	      APPEND_NUMBER_CONVERSION(bit, x, "%x");
	    } break;
	    case 'X': {
	      unsigned int X = va_arg(va, unsigned int);
	      APPEND_NUMBER_CONVERSION(bit, X, "%X");
	    } break;
	      
	    default:
	      throw acpException(aErrParam, "illegal formating type");
	      break;
	  } // switch
	  
	  for (int i = bit.length(); i < zpad; i++)
	    result += '0';
	  
	  result += bit;
	}
	break;
	
      case '\\':
	bEscape = true;
	// fallthrough
	
      default:
	result += *f;
	break;
	
    } // switch
    f++;
    bEscape = false;
  } // while *f

  va_end(va);

  assume(result);

  return m_pStorage;
}


/////////////////////////////////////////////////////////////////////
// copyToBuffer method

void 
acpString::copyToBuffer(char* buffer, 
			const size_t max) const
{
  if (buffer) {
    size_t space = max - 1;
    size_t size = (space < m_length) ? space : m_length;
    aMemCopy(buffer, m_pStorage, size);
    buffer[size] = 0;
#ifdef aDEBUG
    // to be sure the space is handled properly, we slam values into
    // the entire advertised buffer which will cause a problem if 
    // the space is not sent in properly.
    for (size_t i = size + 1; i < max; i++)
      buffer[i] = (char)0xEE;
#endif
  }
}


/////////////////////////////////////////////////////////////////////
// assume method
//
// This co-opts the already allocated from the passed-in string
// and sets the old string up for clean deletion.  The passed in
// string is dead after use and should no longer be accessed.

void 
acpString::assume(acpString& s)
{
  // free the old storage
  aMemFree(m_pStorage);

  // grab the storage from the passed in string
  m_pStorage = s.m_pStorage;
  m_nStorageCapacity = s.m_nStorageCapacity;
  m_length = s.m_length;

  // invalidate the passed in string
  s.m_pStorage = NULL;
#ifdef aDEBUG
  s.m_nStorageCapacity = 0;
  s.m_length = 0;
#endif
}


#ifdef aTESTSTRING
#include "aMemLeakDebug.h"
#include <sys/time.h>

#define BIGSTRINGSIZE 10000

void stressTests(const char* pBigString);
void stressTests(const char* pBigString)
{
  {
    acpString foo(pBigString);
    foo += pBigString;
    for (int i = 0; i < BIGSTRINGSIZE; i++)
      foo += 'b';
    int sum;
    for (int i = 0; i < BIGSTRINGSIZE; i++)
      sum += foo.length();
  }
  {
    acpString bar("test");
    for (int i = 0; i < BIGSTRINGSIZE; i++) {
      acpString foo(bar);
      bar += " this";
      bar.capitalize();
      if (bar.endsWith("this")) {
	bar.lowercase();
	bar.capitalize();
      }
    }
  }
  {
    acpString foo("foo");
    acpString bar("bar");
    for (int i = 0; i < BIGSTRINGSIZE; i++) {
      if (foo != bar)
	foo += bar;
    }
  }
}


/////////////////////////////////////////////////////////////////////
// tests
//
// compile on Unix with:
/*
  g++ -I ../../../aInclude -I../../aCommon -DaUNIX -DaTESTSTRING \
  -DaDEBUG -Wall ../../aCommon/aOSDefs.c ../../aCommon/aMemLeakDebug.c \
  acpException.cpp acpString.cpp
 */



int main(const int argc, const char* argv[])
{
  printf("testing acpString\n");
  unsigned int errors = 0;

  try {
    // construction
    {
      acpString foo(acpString("my new string"));
      if (foo != "my new string") {
	printf("  ERROR: copy construction\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }
    {
      acpString foo(100);
      if (foo != "100") {
	printf("  ERROR: int construction\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }
    {
      acpString foo((char)4);
      if (foo != "4") {
	printf("  ERROR: char construction\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }
    {
      acpString foo((unsigned char)0xFF);
      if (foo != "255") {
	printf("  ERROR: unsigned char construction\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }
    

    // assignments
    {
      const char* string1 = "my string";
      acpString foo;
      foo = string1;
      if (foo != string1) {
	printf("  ERROR: assignments\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      if (foo != (char*)string1) {
	printf("  ERROR: assignments\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      char* string2 = "another that is longer";
      foo = string2;
      if (foo != string2) {
	printf("  ERROR: assignments\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }

    
    // concatenations
    {
      acpString foo("starting");
      if (foo != "starting") {
	printf("  ERROR: concatenations (char*)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (const char*)" string ";
      if (foo != "starting string ") {
	printf("  ERROR: concatenations (const char*)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (const char)'c';
      if (foo != "starting string c") {
	printf("  ERROR: concatenations (char)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (int)33;
      if (foo != "starting string c33") {
	printf("  ERROR: concatenations (int)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (unsigned int)44;
      if (foo != "starting string c3344") {
	printf("  ERROR: concatenations (unsigned int)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (long)5555;
      if (foo != "starting string c33445555") {
	printf("  ERROR: concatenations (long)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (unsigned long)66666;
      if (foo != "starting string c3344555566666") {
	printf("  ERROR: concatenations (unsigned long)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (float)2.3;
      if (foo != "starting string c33445555666662.3") {
	printf("  ERROR: concatenations (float)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (float)0.99999999999;
      if (foo != "starting string c33445555666662.31.0") {
	printf("  ERROR: concatenations (float)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += (float)8.80008;
      if (foo != "starting string c33445555666662.31.08.80008") {
	printf("  ERROR: concatenations (float)\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }

    // trim
    {
      acpString foo("");
      foo.trim();
      if (foo != "") {
	printf("  ERROR: trim error on \"\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = " ";
      foo.trim();
      if (foo != "") {
	printf("  ERROR: trim error on \" \"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "a";
      foo.trim();
      if (foo != "a") {
	printf("  ERROR: trim error on \"a\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = " a";
      foo.trim();
      if (foo != "a") {
	printf("  ERROR: trim error on \"a\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "  a";
      foo.trim();
      if (foo != "a") {
	printf("  ERROR: trim error on \"a\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "a ";
      foo.trim();
      if (foo != "a") {
	printf("  ERROR: trim error on \"a\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "a  ";
      foo.trim();
      if (foo != "a") {
	printf("  ERROR: trim error on \"a\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "bob";
      foo.trim();
      if (foo != "bob") {
	printf("  ERROR: trim error on \"bob\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "bob  ";
      foo.trim();
      if (foo != "bob") {
	printf("  ERROR: trim error on \"bob\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "  bob";
      foo.trim();
      if (foo != "bob") {
	printf("  ERROR: trim error on \"bob\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
      foo = "  bob  ";
      foo.trim();
      if (foo != "bob") {
	printf("  ERROR: trim error on \"bob\"\n");
	printf("   got \"%s\"\n", (char*)foo);
	errors++;
      }
    }
      
    // nasties
    {
      int n = 3;
      acpString foo;
      foo = "Channel ";
      foo += n;
      char* bar = (char*)foo;
      if (foo != "Channel 3") {
	printf("  ERROR: nasties channel\n");
	printf("   got %s and %s\n", (char*)foo, bar);
	errors++;
      }
    }

    {
      acpString foo("knick");
      foo = foo;
      if (foo != "knick") {
	printf("  ERROR: nasties foo = foo\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
      foo += foo;
      if (foo != "knickknick") {
	printf("  ERROR: nasties foo += foo\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }

    {
      int i;
      acpString foo;
      char buf[27];
      for (i = 0; i < 26; i++)
	buf[i] = 'A' + i;
      buf[i] = 0;
      foo = "Vendor: ";
      foo += &buf[23];
      if (foo != "Vendor: XYZ") {
	printf("  ERROR: nasties foo += &buf[23]\n");
	printf("   got %s\n", (char*)foo);
	errors++;
      }
    }

    printf("  %d errors\n", errors);

    if (!errors) {
      printf("  stress testing");
      fflush(stdout);
      struct timeval before;
      struct timeval after;
      struct timezone tz;

      // glob up a large string to use in stress testing
      char* pBigString = (char*)aMemAlloc(BIGSTRINGSIZE + 1);
      if (!pBigString) {
	printf("unable to allocate testing memory\n");
	exit(1);
      }
      int i;
      for (i = 0; i < BIGSTRINGSIZE; i++)
	pBigString[i] = 'a';
      pBigString[i] = 0;

      gettimeofday(&before, &tz);

      for (i = 0; i < 20; i++) {
	stressTests(pBigString);
	printf(".");
	fflush(stdout);
      }

      aMemFree(pBigString);

      gettimeofday(&after, &tz);

      unsigned long lBefore = (unsigned long)((before.tv_sec % 10000) 
					      * 1000L + before.tv_usec / 1000L);
      unsigned long lAfter = (unsigned long)((after.tv_sec % 10000) 
					      * 1000L + after.tv_usec / 1000L);
      printf("\ncompleted in %f seconds\n", (lAfter - lBefore) / 1000.0f);
    }
  } catch (const acpException& exception) {
    printf("exception caught: %s\n", (const char*)exception);
    errors = -1;
  }

  aLeakCheckCleanup();

  return errors;
  
} // tests
#endif // aTESTSTRING
