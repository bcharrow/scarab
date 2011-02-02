/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetServoMessage.cpp                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GetServoMessage object.      //
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

#include "acpGetServoMessage.h"
#include "aServo.h"
#include "aUtil.h"



/////////////////////////////////////////////////////////////////////
// acpGetServoPositionMessage process method
//

void acpGetServoPositionMessage::process()
{
  aErr err;
  unsigned char pos;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  err = aServo_GetPosition(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_index,
  			 &pos);

  if (err == aErrNone) {      		         
    *m_pfValue = ((float)pos) / 255.0f;
  }

} // acpGetServoPositionMessage::process method



/////////////////////////////////////////////////////////////////////
// acpGetServoConfigMessage process method
//

void acpGetServoConfigMessage::process()
{
  aErr err;
  unsigned char config;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  err = aServo_GetConfig(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_index,
  			 &config);

  if (err == aErrNone) {      		         
    *m_pcValue = config;
  }

} // acpGetServoConfigMessage::process method



/////////////////////////////////////////////////////////////////////
// acpGetServoLimitsMessage process method
//

void acpGetServoLimitsMessage::process()
{
  aErr err;
  unsigned char limitbytes[4];

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  err = aServo_GetLimits(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_index,
  			 limitbytes);

  if (err == aErrNone) {      		         
    *m_psValue = aUtil_RetrieveShort((char*)limitbytes);
  }

} // acpGetServoLimitsMessage::process method
