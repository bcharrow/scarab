/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetOdometerMessage.cpp                                 //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GetOdometerMessage object.   //
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

#include "acpGetOdometerMessage.h"
#include "aMotion.h"



/////////////////////////////////////////////////////////////////////
// acpGetOdometerMessage process method
//

void acpGetOdometerMessage::process()
{
  aErr err;
  char data[aSTEMMAXPACKETBYTES];

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  aBZero(data, 4);
  
  err = aMotion_GetEnc32(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_index,
  			 data);

  if (err == aErrNone) {      		         

    float fDistance;
    long n = 0;

    n |= (((int)data[3]) & 0xFF);
    n |= (((int)data[2]) & 0xFF) << 8;
    n |= (((int)data[1]) & 0xFF) << 16;
    n |= (((int)data[0]) & 0xFF) << 24;
    fDistance = (float)n;

    *m_fValue = fDistance / 
    		(float)m_pcGarcia->ticksPerUnitDistance();
  }

} // acpGetOdometerMessage::process method
