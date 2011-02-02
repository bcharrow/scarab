/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: stdio_acpGarciaAgent.cpp                                  //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GarciaAgent application      //
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
#include "stdio_acpGarciaAgent.h"


/////////////////////////////////////////////////////////////////////

stdio_acpGarciaAgent::stdio_acpGarciaAgent() :
  m_bDone(false)
{
  aErr err;
  err = aStream_Create_STDIO_Console_Output(m_ioRef, &m_logView);
  aAssert(err == aErrNone);

} // stdio_acpGarciaAgent::stdio_acpGarciaAgent



/////////////////////////////////////////////////////////////////////

stdio_acpGarciaAgent::~stdio_acpGarciaAgent()
{
  if (m_logView)
    aStream_Destroy(m_ioRef, m_logView, NULL);
}



/////////////////////////////////////////////////////////////////////

void stdio_acpGarciaAgent::run()
{
  while (!m_bDone) {
    TimeSlice();
    aIO_MSSleep(m_ioRef, 2, NULL);
  }
}



/////////////////////////////////////////////////////////////////////

void stdio_acpGarciaAgent::setStatusText(const char* msg)
{
}



/////////////////////////////////////////////////////////////////////

void stdio_acpGarciaAgent::updateHeartbeat()
{
} // stdio_acpGarciaAgent updateHeartbeat



/////////////////////////////////////////////////////////////////////

void stdio_acpGarciaAgent::cmdExit()
{
  m_bDone = true;

} // stdio_acpGarciaAgent cmdExit
