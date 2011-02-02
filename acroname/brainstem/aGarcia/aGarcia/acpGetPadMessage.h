/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetPadMessage.h                                        //
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

#ifndef _acpGetPadMessage_H_
#define _acpGetPadMessage_H_

#include "acpGarciaInternal.h"

class acpGetPadMessage :
  public acpMessage
{
  public:
  			acpGetPadMessage(acpGarciaInternal* pGarcia,
  					 const unsigned char address,
  					 const unsigned char offset,
  					 char* pValue) :
  			    m_address(address),
  			    m_offset(offset),
  			    m_pValue(pValue),
  			    m_pcGarcia(pGarcia) {}

    void		process();

    unsigned char	m_address;
    unsigned char	m_offset;
    char*		m_pValue;

  private:

    acpGarciaInternal*	m_pcGarcia;
};

#endif // _acpGetPadMessage_H_
