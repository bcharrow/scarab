/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetSonarMessage.cpp                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GetSonarMessage object.      //
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

#include "acpGetSonarMessage.h"
#include "aSRF08.h"
#include "aUtil.h"



/////////////////////////////////////////////////////////////////////
// acpGetSonarRangeMessage process method
//

void acpGetSonarRangeMessage::process()
{
  aErr err;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  err = aSRF08_GetRange(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_addr,
  			 m_units,
  			 m_psBuff,
  			 (unsigned char)m_nvals);

  if (err == aErrNone) {      		         
    *m_pnValue = (int)m_psBuff[0];
  }

} // acpGetSonarRangeMessage::process method



/////////////////////////////////////////////////////////////////////
// acpGetSonarLightMessage process method
//

void acpGetSonarLightMessage::process()
{
  aErr err;
  unsigned char val;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  err = aSRF08_GetLight(m_pcGarcia->m_stemRef,
  			 m_module,
  			 m_addr,
  			 &val);

  if (err == aErrNone) {      		         
    *m_pnValue = (int)val;
  }

} // acpGetSonarLightMessage::process method
