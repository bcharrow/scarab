/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: acpPacket.h                                               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of stem packet object.		   */
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

#ifndef _acpPacket_H_
#define _acpPacket_H_

#define	kHasAddress 	0x01
#define kHasLength	0x02

#include "acpMessage.h"
#include "aStem.h"

#ifdef aOLDSCHOOL
#undef aSTEM_EXPORT
#define aSTEM_EXPORT
#endif

class aSTEM_EXPORT acpPacket : 
  public acpMessage {
  public:
			acpPacket(aStemLib stemRef,
				  aPacketRef packetRef);
  			acpPacket() : 
			  m_packetRef(NULL),
			  m_flags(0) {}
    virtual		~acpPacket();
  
    bool		isFull()
    			  {return (m_flags == (kHasAddress | kHasLength))
    			  	  && (m_length == m_data[1]);}
    bool		isEmpty()
    			  {return !(m_flags & kHasAddress);}
    
    void		reset()
    			  {m_flags = 0;}

    virtual void	process() {} 
  
    aPacketRef		getRef() { return m_packetRef; }
    unsigned char	getAddress();
//    unsigned char	getCMD();
    unsigned char	getLength();
    const char* 	getData();

    void		addByte(const char byte);
    aErr		writeToStream(aStreamRef stream);

  private:
    aStemLib		m_stemRef;
    aPacketRef          m_packetRef;
    char		m_flags;
    char		m_length;
    char		m_data[aSTEMMAXPACKETBYTES + 2];
};

#endif /* _acpPacket_H_ */
