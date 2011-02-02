/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aRobotPackager_main.cpp       		                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: command line version of the compiler.              */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2008. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "acpException.h"
#include "aMemLeakDebug.h"
#include "acpRobotPackager.h"
#include "acpException.h"


// leaked object checking
#ifdef aDEBUG
void* operator new(size_t size) throw() {return aMemAlloc(size);}
void operator delete(void *p) throw() {return aMemFree(p);}
#endif /* aDEBUG */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * main 
 */

int main(
  const int argc, 
  const char* argv[]
)
{
  int result = 1;
  try {

    acpRobotPackager packager;
    result = packager.handleCommands(argc, argv);
    /*MJK result = 0; */

  } catch (const acpException& exception) {
    printf("unhandled exception %s\n", (const char*)exception);
  } 

  aLeakCheckCleanup();
  
  /*MJK return -1; */
  return result;

} /* end of main */
