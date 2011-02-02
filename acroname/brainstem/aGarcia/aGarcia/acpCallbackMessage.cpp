/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpCallbackMessage.cpp                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the CallbackMessage object that  //
//              handles callbacks from the garcia thread to the    //
//              caller.                                            //
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

#include "acpCallbackMessage.h"



/////////////////////////////////////////////////////////////////////
// acpCallbackMessage process method
//

void acpCallbackMessage::process()
{
  // be sure we aren't in the thread's context
  aAssert(!m_pcGarcia->m_pcThread->isThreadContext());

  aAssert(m_pcCallback);
  m_pcCallback->call();

  m_pcCallback->release();

  // not running means this was a completion callback
  if (!m_pcBehavior->isRunning())
    delete m_pcBehavior;

} // acpCallbackMessage::process method
