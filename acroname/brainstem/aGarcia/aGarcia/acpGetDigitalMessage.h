/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetDigitalMessage.h                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the digital IO message sent to the   //
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

#ifndef _acpGetDigitalMessage_H_
#define _acpGetDigitalMessage_H_

#include "acpGarciaInternal.h"

class acpGetDigitalMessage :
  public acpMessage
{
  public:

  			acpGetDigitalMessage(acpGarciaInternal* pGarcia,
  					 const unsigned char module,
  					 const unsigned char index,
  					 int* pnValue) :
  			    m_module(module),
  			    m_index(index),
  			    m_nValue(pnValue),
  			    m_pcGarcia(pGarcia) {}

    void		process();

    unsigned char	m_module;
    unsigned char	m_index;
    int*		m_nValue;

  private:

    acpGarciaInternal*	m_pcGarcia;
};

#endif // _acpGetDigitalMessage_H_
