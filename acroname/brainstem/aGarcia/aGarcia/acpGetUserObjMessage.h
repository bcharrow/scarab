/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetUserObjMessage.h                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the UserObj messages sent to the     //
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

#ifndef _acpGetUserObjMessage_H_
#define _acpGetUserObjMessage_H_

#include "acpGarciaInternal.h"

class acpGetUserObjMessage :
  public acpMessage
{
  public:

  			acpGetUserObjMessage(
  			  acpGarciaInternal* pGarcia,
  			  const unsigned char module,
  			  const unsigned char size,
  			  const unsigned char replybyte,
  			  const unsigned long timeout,
  			  char* data,
  			  int* pnValue) :
  			    m_module(module),
  			    m_size(size),
  			    m_replybyte(replybyte),
  			    m_timeout(timeout),
  			    m_data(data),
  			    m_pnValue(pnValue),
  			    m_pcGarcia(pGarcia) {}

    void		process();

    unsigned char	m_module;
    unsigned char	m_size;
    unsigned char	m_replybyte;
    unsigned long	m_timeout;
    char*		m_data;
    int*		m_pnValue;

  private:

    acpGarciaInternal*	m_pcGarcia;
};

#endif // _acpGetUserObjMessage_H_