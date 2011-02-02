/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aCrypt.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent		   */
/*		licensing encryption.				   */
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

#include "aTEA.h"
#include "aUtil.h"
#include "aCrypt.h"

#define CRYPTWIDTH		20

#define encodingString "Please don't copy this sofware. It is copyrighted by Acroname Inc."

static char display[16] = {'5', 'a', 'U', 'o', 
		           '0', 'O', 'T', 'j',
		           'Q', 'L', 'e', 'w',
		           'z', 'M', 'x', 'J'};
		           
static unsigned int invDisplay(char c);
static unsigned int invDisplay(char c)
{
  unsigned int i;

  for (i = 0; i < 16; i++) {
    if (display[i] == c)
     return i;
  }

  return 0;
}


#ifdef aGENCRYPT

#if 0
void show(const char* text);
void show(const char* text) {
  char* p = (char*)text;
  while (*p) {
    printf("%c-%d ", *p, *p);
    *p++;
  }
  printf("\n");
}
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aCrypt_Encode
 *
 * output in lines of CRYPTWIDTH characters
 */

aErr aCrypt_Encode(const char* pFirstName,
		   const char* pLastName,
		   const char* pCustomerID,
		   const char* pVendorName,
		   aStreamRef output)
{
  aErr cryptErr = aErrNone;
  char data[CRYPTLEN];
  unsigned int i, j, k;
  char line[CRYPTWIDTH + 1];
  char* p;
  char crypt[CRYPTLEN] = {0x1D,0x21,0x28,0x2C,0x3E,0x28,0x6D,0x29,0x22,0x23,0x6A,0x39,0x6D,0x2E,0x22,0x3D,0x34,0x6D,0x39,0x25,0x24,0x3E,0x6D,0x3E,0x22,0x2B,0x3A,0x2C,0x3F,0x28,0x63,0x6D,0x4,0x39,0x6D,0x24,0x3E,0x6D,0x2E,0x22,0x3D,0x34,0x3F,0x24,0x2A,0x25,0x39,0x28,0x29,0x6D,0x2F,0x34,0x6D,0xC,0x2E,0x3F,0x22,0x23,0x2C,0x20,0x28,0x6D,0x4,0x23,0x2E,0x63,0x1D,0x21,0x28,0x2C,0x3E,0x28,0x6D,0x29,0x22,0x23,0x6A,0x39,0x6D,0x2E,0x22,0x3D,0x34,0x6D,0x39,0x25,0x24,0x3E,0x6D,0x3E,0x22,0x2B,0x3A,0x2C,0x3F,0x28,0x63,0x6D,0x4,0x39};
  short checksum = 0;

#if 0
  /* xor the encoding string into a buffer */
  aStringCopy((char*)data, encodingString);
  k = aStringLen((char*)data);
  j = 0;
  i = 0;
  while (i < CRYPTLEN) {
    crypt[i] = (char)(data[j] ^ 'M');
    i++;
    j++;
    if (j == k)
      j = 0;
  }
  
  printf("char crypt[CRYPTLEN] = {");
  for (i = 0; i < CRYPTLEN; i++) {
    printf("0x%X,", crypt[i]);
  }
  printf("};");
#endif

  /* copy the strings into the data array */
  for (i = 2, p = (char*)pFirstName; i < CRYPTNAMELEN + 2; i++)
    data[i] = (char)((*p) ? *p++ : 0);
  for (i = CRYPTNAMELEN + 2, p = (char*)pLastName; 
       i < CRYPTNAMELEN*2 + 2; i++)
    data[i] = (char)((*p) ? *p++ : 0);
  for (i = CRYPTNAMELEN*2 + 2, p = (char*)pCustomerID; 
       i < CRYPTLEN; i++)
    data[i] = (char)((*p) ? *p++ : 0);
  for (i = CRYPTNAMELEN*2 + CRYPTIDLEN + 2, p = (char*)pVendorName; 
       i < CRYPTLEN; i++)
    data[i] = (char)((*p) ? *p++ : 0);

  /* compute the checksum */
  for (i = 2; i < CRYPTLEN; i++)
    checksum += data[i];
  aUtil_StoreShort(data, checksum);
 
  /* xor the crypt with the data */
  for (i = 0; (cryptErr == aErrNone) && (i < CRYPTLEN);) {
    for (j = 0; (cryptErr == aErrNone) 
    		&& (j < CRYPTWIDTH) 
    		&& (i < CRYPTLEN); 
    	 j += 2, i++) {
      k = data[i];
      k = k ^ i ^ crypt[i];
      line[j] = display[k & 0x0F];
      line[j+1] = display[(k & 0xF0) >> 4];
    }
    line[CRYPTWIDTH] = '\n';
    aStream_Write(aStreamLibRef(output), output, line, 
    		  CRYPTWIDTH + 1, &cryptErr);
  }

  return cryptErr;

} /* aCrypt_Encode */

#endif /* aGENCRYPT */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aCrypt_Decode
 */

aErr aCrypt_Decode(const aStreamRef input,
		   char* pFirstName,
		   char* pLastName,
		   char* pCustomerID,
		   char* pVendorName)
{
  aErr cryptErr = aErrNone;
  unsigned int i, j, k;
  char data[CRYPTLEN];
  char line[CRYPTWIDTH+1];
  char* p;
  char crypt[CRYPTLEN] = {0x1D,0x21,0x28,0x2C,0x3E,0x28,0x6D,0x29,0x22,0x23,0x6A,0x39,0x6D,0x2E,0x22,0x3D,0x34,0x6D,0x39,0x25,0x24,0x3E,0x6D,0x3E,0x22,0x2B,0x3A,0x2C,0x3F,0x28,0x63,0x6D,0x4,0x39,0x6D,0x24,0x3E,0x6D,0x2E,0x22,0x3D,0x34,0x3F,0x24,0x2A,0x25,0x39,0x28,0x29,0x6D,0x2F,0x34,0x6D,0xC,0x2E,0x3F,0x22,0x23,0x2C,0x20,0x28,0x6D,0x4,0x23,0x2E,0x63,0x1D,0x21,0x28,0x2C,0x3E,0x28,0x6D,0x29,0x22,0x23,0x6A,0x39,0x6D,0x2E,0x22,0x3D,0x34,0x6D,0x39,0x25,0x24,0x3E,0x6D,0x3E,0x22,0x2B,0x3A,0x2C,0x3F,0x28,0x63,0x6D,0x4,0x39};
  short checksum = 0;
  short filesum;
  

  /* decode the display characters and xor back*/
  for (i = 0; (cryptErr == aErrNone) && (i < CRYPTLEN);) {
    aStream_ReadLine(aStreamLibRef(input), input, line, 
    		     CRYPTWIDTH + 1, &cryptErr);
    for (j = 0; (cryptErr == aErrNone) && (j < CRYPTWIDTH); j += 2, i++) {
      k = line[j];
      k = invDisplay(line[j]) | (invDisplay(line[j+1]) << 4);
      k = crypt[i] ^ i ^ k;
      data[i] = (char)k;
    }
  }

  /* compute the checksum */
  for (i = 2; i < CRYPTLEN; i++)
    checksum += data[i];
  filesum = aUtil_RetrieveShort(data);
  if (checksum != filesum)
    return aErrParam;

  /* copy out the results */
  /* copy the strings into the data array */
  for (i = 2, p = (char*)pFirstName; i < CRYPTNAMELEN + 2; i++) {
    if (data[i])
      *p++ = data[i]; 
    else {
      *p = 0;
      break;
    }
  }
  for (i = CRYPTNAMELEN + 2, p = (char*)pLastName; 
       i < CRYPTNAMELEN*2 + 2; i++) {
    if (data[i])
      *p++ = data[i]; 
    else {
      *p = 0;
      break;
    }
  }
  for (i = CRYPTNAMELEN*2 + 2, p = (char*)pCustomerID; 
       i < CRYPTLEN; i++) {
    if (data[i])
      *p++ = data[i]; 
    else {
      *p = 0;
      break;
    }
  }
  for (i = CRYPTNAMELEN*2 + CRYPTIDLEN + 2, p = (char*)pVendorName; 
       i < CRYPTLEN; i++) {
    if (data[i])
      *p++ = data[i]; 
    else {
      *p = 0;
      break;
    }
  }

  return cryptErr;

} /* aCrypt_Decode */

#ifdef aGENCRYPT

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * main
 */

#include "aStream_STDIO_Console.h"

int main(int argc, char* argv[]);
int main(int argc, char* argv[])
{
  aErr err = aErrNone;
  aStreamRef output;
  aIOLib ioLib;
  
  if (argc < 2)
    goto usage;

  if (err == aErrNone)
    aIO_GetLibRef(&ioLib, &err);
    
  if (err == aErrNone)
    err = aStream_Create_STDIO_Console_Output(ioLib, &output);

  if (aStringCompare(argv[1], "-e") == 0) {

    if (argc != 6)
      goto usage;

    if (err == aErrNone)
      err = aCrypt_Encode(argv[2], argv[3], argv[4], argv[5], output);

  } else if (aStringCompare(argv[1], "-d") == 0) {
    char pFirstName[CRYPTNAMELEN + 1];
    char pLastName[CRYPTNAMELEN + 1];
    char pCustomerID[CRYPTNAMELEN + 1];
    char pVendorName[CRYPTVENDORLEN + 1];
    aStreamRef input;
 
    if (argc != 3)
      goto usage;
 
    if (err == aErrNone)
      aStream_CreateFileInput(ioLib, argv[2], aFileAreaBinary, 
      			      &input, &err);

    if (err == aErrNone)
      err = aCrypt_Decode(input, pFirstName, 
      			  pLastName, pCustomerID,
      			  pVendorName);

    if (err == aErrNone)
      aStream_Destroy(ioLib, input, &err);

    if (err == aErrNone) {
      printf("First Name: %s\n", pFirstName);
      printf("Last Name: %s\n", pLastName);
      printf("Customer ID: %s\n", pCustomerID);
      printf("Vendor Name: %s\n", pVendorName);
    }
  }

  if (err == aErrNone)
    aStream_Destroy(ioLib, output, &err);

  aIO_ReleaseLibRef(ioLib, &err);

  return 0;

usage:
  printf("ERROR\n");
  printf("usage: crypt -e firstname lastname customerid vendorname\n");
  printf("       crypt -d filename\n");

  return 1;

} /* main */

#endif /* aGENCRYPT */
