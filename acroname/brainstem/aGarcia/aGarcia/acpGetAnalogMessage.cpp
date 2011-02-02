/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetAnalogMessage.cpp                                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GetAnalogMessage object.     //
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

#include "acpGetAnalogMessage.h"
#include "aAnalog.h"



/////////////////////////////////////////////////////////////////////
// acpGetAnalogMessage process method
//

void acpGetAnalogMessage::process()
{
  aErr err;
  int nRawValue;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  err = aAnalog_ReadInt(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_index,
  			 &nRawValue);

  if (err == aErrNone) {      		         

    // convert 10-bit value to actual voltage
    *m_fValue = 5.0f * (nRawValue / 1023.0f);
  }

} // acpGetAnalogMessage::process method
