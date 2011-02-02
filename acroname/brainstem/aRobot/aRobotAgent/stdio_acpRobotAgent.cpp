/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: stdio_acpRobotAgent.cpp                                  //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the RobotAgent application      //
//              object.                                            //
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

#include "aStream_STDIO_Console.h"
#include "stdio_acpRobotAgent.h"


/////////////////////////////////////////////////////////////////////

stdio_acpRobotAgent::stdio_acpRobotAgent() :
  acpRobotAgent(),
  m_bDone(false)
{
  aErr err;
  err = aStream_Create_STDIO_Console_Output(m_ioRef, &m_logView);
  aAssert(err == aErrNone);

} // stdio_acpRobotAgent::stdio_acpRobotAgent



/////////////////////////////////////////////////////////////////////

stdio_acpRobotAgent::~stdio_acpRobotAgent()
{
  if (m_logView)
    aStream_Destroy(m_ioRef, m_logView, NULL);
}



/////////////////////////////////////////////////////////////////////

void stdio_acpRobotAgent::run()
{
  while (!m_bDone) {
    TimeSlice();
    aIO_MSSleep(m_ioRef, 2, NULL);
  }
}



/////////////////////////////////////////////////////////////////////

void stdio_acpRobotAgent::setStatusText(
  const char* msg)
{
}



/////////////////////////////////////////////////////////////////////

void stdio_acpRobotAgent::updateHeartbeat()
{
} // stdio_acpRobotAgent updateHeartbeat



/////////////////////////////////////////////////////////////////////

void stdio_acpRobotAgent::cmdExit()
{
  m_bDone = true;

} // stdio_acpRobotAgent cmdExit
