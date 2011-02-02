/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aPacket.h                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent BrainStem */
/*		communication layer.    			   */
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

#ifndef _aPacket_H_
#define _aPacket_H_


#include "aCmd.tea"

#include "aStem.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * packet status flags
 */

#define aPacketHasAddr    0x01
#define aPacketHasSize    0x02
#define aPacketFromStem   0x04

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * type definitions
 */

typedef struct aPacket {
  unsigned char		address;  /* from/to which iic module */
  unsigned char		dataSize; /* length of following data */
  unsigned char		curSize;  /* current number of data input */
  unsigned char		status;
  char			data[aSTEMMAXPACKETBYTES];
  int			check;
  struct aPacket*	pNext;
} aPacket;

typedef struct aPacketList {
  int 			nSize;
  aMemPoolRef		packetPoolRef;
  aPacket*		pHead;
  aPacket*		pTail;
} aPacketList;

#define aPACKETCHECK  0xCEED

#define aVALIDPACKET(p) if (!p || 				   \
			 (((aPacket*)p)->check != aPACKETCHECK))   \
                           err = aErrParam;

#define aPACKET_IS_HB(p) ((p)                                      \
			  && ((p)->dataSize == 2)                  \
			  && ((p)->data[0] == cmdHB))

#define aPACKET_IS_RELAY(p) ((p)                                   \
			     && ((p)->dataSize >= 2)               \
			     && ((p)->data[0] == cmdSER_TOIIC))

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * library internal packet routine definitions
 */

#ifdef __cplusplus
extern "C" {
#endif

void 
aPacket_Init(aPacket* pPacket,
	     unsigned char initflags);

int 
aPacket_Size(aPacket* pPacket);

int 
aPacket_BytesSoFar(aPacket* pPacket);

aBool 
aPacket_Complete(aPacket* pPacket);

aErr 
aPacket_AddByte(aPacket* pPacket, 
		unsigned char byte);

aErr 
aPacket_WriteToStream(aPacket* pPacket,
		      aStreamRef stream);




aErr  
aPacketList_Create(aMemPoolRef packetPoolRef,
		   aPacketList** ppPacketList);

aErr 
aPacketList_AddCopy(void* vpStem,
		    aPacketList* pPacketList,
		    aPacket* pPacket);
aErr 
aPacketList_GetFirst(aPacketList* pPacketList,
		     aPacketFilter filterProc,
		     void* filterRef,
		     aPacket** ppPacket);
aErr 
aPacketList_Destroy(void* vpStem,
		    aPacketList* pPacketList);

#ifdef __cplusplus 
}
#endif

#endif /* _aPacket_H_ */

