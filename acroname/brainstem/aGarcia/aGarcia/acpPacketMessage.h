/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPacketMessage.h                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the packet message sent to the       //
//              private thread in the Garcia API library object.   //
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

#ifndef _acpPacketMessage_H_
#define _acpPacketMessage_H_

#include "acpGarciaInternal.h"

class acpPacketMessage :
  public acpMessage
{
  public:
  			acpPacketMessage(acpGarciaInternal* pGarcia,
  					 const unsigned char address,
  					 const unsigned char length,
  					 const char* data) :
  			    m_address(address),
  			    m_length(length),
  			    m_pcGarcia(pGarcia)
  			  { aMemCopy(m_data, data, length); }

    void		process()
    			  { m_pcGarcia->handlePacketOut(this); }

    unsigned char	m_address;
    unsigned char	m_length;
    char		m_data[aSTEMMAXPACKETBYTES];

  private:
    acpGarciaInternal*	m_pcGarcia;
};

#endif // _acpPacketMessage_H_
