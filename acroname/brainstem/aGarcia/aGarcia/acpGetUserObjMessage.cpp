/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetUserObjMessage.cpp                                  //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GetUserObjMessage object.    //
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

#include "acpGetUserObjMessage.h"
#include "aStemCore.h"
#include "aUtil.h"



/////////////////////////////////////////////////////////////////////
// acpGetUserObjMessage process method
//

void acpGetUserObjMessage::process()
{
  aErr err = aErrNone;
  aPacketRef packet;
  unsigned char address;
  unsigned char length;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  
  /* build up the packet */
  if (err == aErrNone)
    aPacket_Create(m_pcGarcia->m_stemRef, 
    		   m_module, 
    		   m_size, 
    		   m_data, 
    		   &packet, 
    		   &err);
    
  /* send it */
  if (err == aErrNone)
    aStem_SendPacket(m_pcGarcia->m_stemRef, packet, &err);
    
  if (m_replybyte) {

    /* wait for the reply */
    if (err == aErrNone)
      aStem_GetPacket(m_pcGarcia->m_stemRef, 
    		    aStemCore_CmdFilter, 
    		    (void*)((int)m_replybyte), 
      		    m_timeout, 
      		    &packet,
      		    &err);
    
    /* get the reply packet data */
    if (err == aErrNone)
      aPacket_GetData(m_pcGarcia->m_stemRef, 
    		    packet, 
    		    &address,
      		    &length, 
      		    m_data, 
      		    &err);

    /* clean up the packet */
    if (err == aErrNone) {
      aPacket_Destroy(m_pcGarcia->m_stemRef, packet, NULL);
    }
  }
  
  *m_pnValue = (int)err;
 
} // acpGetUserObjMessage::process method
