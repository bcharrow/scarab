/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotPacketMessage.h                                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the packet message sent to the       //
//              private thread in the Robot API library object.    //
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

#ifndef _acpRobotPacketMessage_H_
#define _acpRobotPacketMessage_H_

#include "acpRobotInternal.h"

class acpRobotPacketMessage :
  public acpMessage
{
  public:
  			acpRobotPacketMessage(acpRobotInternal* pRobot,
  					 const int nLinkID,
  					 const unsigned char address,
  					 const unsigned char length,
  					 const char* data) :
  			    m_nLinkID(nLinkID),
  			    m_address(address),
  			    m_length(length),
  			    m_pcRobot(pRobot)
  			  { aMemCopy(m_data, data, length); }

    void		process()
    			  { m_pcRobot->handlePacketOut(this); }

    int			m_nLinkID;
    unsigned char	m_address;
    unsigned char	m_length;
    char		m_data[aSTEMMAXPACKETBYTES];

  private:
    acpRobotInternal*	m_pcRobot;
};

#endif // _acpPacketMessage_H_
