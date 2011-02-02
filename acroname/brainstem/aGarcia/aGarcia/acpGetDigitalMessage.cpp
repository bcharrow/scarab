/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetDigitalMessage.cpp                                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GetDigitalMessage object.     //
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

#include "acpGetDigitalMessage.h"
#include "aDigital.h"



/////////////////////////////////////////////////////////////////////
// acpGetDigitalMessage process method
//

void acpGetDigitalMessage::process()
{
  aErr err;
  int nRawValue;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  err = aDigital_ReadInt(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_index,
  			 &nRawValue);

  if (err == aErrNone) {      		         
    *m_nValue = nRawValue;
  }


} // acpGetDigitalMessage::process method
