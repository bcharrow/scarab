/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aCrypt.h						   */
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


#ifndef _aCrypt_H_
#define _aCrypt_H_

#include "aIO.h"

#define CRYPTNAMELEN		32
#define CRYPTIDLEN		16
#define CRYPTCHCKLEN		2
#define CRYPTVENDORLEN		18
#define CRYPTLEN		CRYPTNAMELEN * 2 + \
				CRYPTIDLEN + \
				CRYPTCHCKLEN + \
				CRYPTVENDORLEN

aErr aCrypt_Encode(const char* pFirstName,
		   const char* pLastName,
		   const char* pCustomerID,
		   const char* pVendorName,
		   aStreamRef output); 

aErr aCrypt_Decode(const aStreamRef input,
		   char* pFirstName,
		   char* pLastName,
		   char* pCustomerID,
		   char* pVendorName);

#endif /* _aCrypt_H_ */
