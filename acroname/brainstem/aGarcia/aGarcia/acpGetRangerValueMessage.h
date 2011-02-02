/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetRangerValueMessage.h                                //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the ranger message sent to the       //
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

#ifndef _acpGetRangerValueMessage_H_
#define _acpGetRangerValueMessage_H_

#include "acpGarciaInternal.h"

class acpGetRangerValueMessage :
  public acpMessage
{
  public:

    typedef enum {
      kEnableCheck,
      kReadA2D
    } eProcessState;

  			acpGetRangerValueMessage(acpGarciaInternal* pGarcia,
  					 const unsigned char id,
  					 const unsigned char code,
  					 acpValue* pValue) :
  			    m_enablemodule(0),
  			    m_cID(id),
  			    m_cCode(code),
  			    m_eState(kEnableCheck),
  			    m_pValue(pValue),
  			    m_pcGarcia(pGarcia) {}

    void		process();

    unsigned char	m_enablemodule;
    unsigned char	m_cID;
    unsigned char	m_cCode;
    eProcessState	m_eState;
    acpValue*		m_pValue;

  private:

    acpGarciaInternal*	m_pcGarcia;
};

#endif // _acpGetRangerValueMessage_H_
