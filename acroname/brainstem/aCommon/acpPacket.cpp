/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: acpPacket.cpp                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of stem packet object.		   */
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

#include "acpPacket.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

acpPacket::acpPacket(aStemLib stemRef,
		     aPacketRef packetRef) :
  m_stemRef(stemRef),
  m_packetRef(packetRef)
{
  m_flags = kHasAddress | kHasLength;
  m_length = 0;
  m_data[0] = 0;
  
} // acpPacket constructor


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

acpPacket::~acpPacket()
{
  if (m_packetRef)
    aPacket_Destroy(m_stemRef, m_packetRef, NULL);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

unsigned char
acpPacket::getAddress()
{
  /* do lazy retrieval of packet data from the internal structure */
  /* this can go away once this file is tucked into the aStem */
  /* library where it can directly access the aPacket structure */
  if (m_packetRef && !m_data[0]) {
    aPacket_GetData(m_stemRef,
		    m_packetRef,
		    (unsigned char*)&m_data[0],
		    (unsigned char*)&m_data[1],
		    &m_data[2],
		    NULL);
    m_length = m_data[1];
  }

  return (unsigned char)m_data[0]; 
}


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

unsigned char 
acpPacket::getCMD()
{
  /* do lazy retrieval of packet data from the internal structure */
  /* this can go away once this file is tucked into the aStem */
  /* library where it can directly access the aPacket structure */
  if (m_packetRef && !m_data[0]) {
    aPacket_GetData(m_stemRef,
		    m_packetRef,
		    (unsigned char*)&m_data[0],
		    (unsigned char*)&m_data[1],
		    &m_data[2],
		    NULL);
    m_length = m_data[1];
  }

  return (unsigned char)m_data[2]; 
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

unsigned char
acpPacket::getLength()
{ 
  /* do lazy retrieval of packet data from the internal structure */
  /* this can go away once this file is tucked into the aStem */
  /* library where it can directly access the aPacket structure */
  if (m_packetRef && !m_data[0]) {
    aPacket_GetData(m_stemRef,
		    m_packetRef,
		    (unsigned char*)&m_data[0],
		    (unsigned char*)&m_data[1],
		    &m_data[2],
		    NULL);
    m_length = m_data[1];
  }

  return (unsigned char)m_length; 
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

const char* 
acpPacket::getData()
{ 
  /* do lazy retrieval of packet data from the internal structure */
  /* this can go away once this file is tucked into the aStem */
  /* library where it can directly access the aPacket structure */
  if (m_packetRef && !m_data[0]) {
    aPacket_GetData(m_stemRef,
		    m_packetRef,
		    (unsigned char*)&m_data[0],
		    (unsigned char*)&m_data[1],
		    &m_data[2],
		    NULL);
    m_length = m_data[1];
  }

  return &m_data[2]; 
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void acpPacket::addByte(const char byte)
{
  if (!(m_flags & kHasAddress)) {
    m_flags |= kHasAddress;
    m_data[0] = byte;
  } else if (!(m_flags & kHasLength)) {
    aAssert(byte <= aSTEMMAXPACKETBYTES);
    m_flags |= kHasLength;
    m_data[1] = byte;
    m_length = 0;
  } else {
    aAssert(m_length < m_data[1]);
    m_data[m_length + 2] = byte;
    m_length++;
  }

} // addByte method


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
aErr acpPacket::writeToStream(aStreamRef stream)
{
  aErr pcktErr = aErrNone;

  aAssert(isFull());
  
  aStream_Write(aStreamLibRef(stream), stream, 
  		m_data, (aMemSize)(m_data[1] + 2), &pcktErr);
 
  // if nobody was listening, just toss the packet
  if (pcktErr == aErrEOF)
    pcktErr = aErrNone;

  m_flags = 0;
  
  return pcktErr;

} // writeToStream method
