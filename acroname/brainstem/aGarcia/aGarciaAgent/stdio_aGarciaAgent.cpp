/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: stdio_aGarciaAgent.cpp                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GarciaAgent example          //
//              application.                                       //
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

#include "aErr.h"
#include "aMemLeakDebug.h"
#include "stdio_acpGarciaAgent.h"

// leaked object checking
#if defined(aDEBUG) && !defined(aWINCE)
void* operator new(size_t size) {return aMemAlloc(size);}
void operator delete(void *p) {aMemFree(p);}
#endif /* aDEBUG */


/////////////////////////////////////////////////////////////////////
// main

int main(
  int argc,
  char* argv[]
)
{
  {
    stdio_acpGarciaAgent app;
    app.run();
  }

  aLeakCheckCleanup();

  return(0);

} // main


