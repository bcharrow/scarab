/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotCallbackMessage.cpp                               //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the callback message object that //
//              handles callbacks from the Robot thread to the     //
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

#include "acpRobotCallbackMessage.h"



/////////////////////////////////////////////////////////////////////
// acpRobotCallbackMessage process method
//

void acpRobotCallbackMessage::process()
{
  // be sure we aren't in the thread's context
  aAssert(!m_pcRobot->m_pcThread->isThreadContext());

  aAssert(m_pcCallback);
  m_pcCallback->call();

  m_pcCallback->release();

  // we could have a case where a primitive executes quickly
  // and is no longer running by the time an exec callback is
  // executed so we use a flag to tell what type of callback it is

  if (m_bCompCB && !m_pcBehavior->isRunning())
    delete m_pcBehavior;

} // acpRobotCallbackMessage::process method
