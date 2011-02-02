/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_acpRobotOSFactory.cpp                                */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: The pure virtual class factory for the Robot lib.  */
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


#include "unix_acpMutex.h"
#include "unix_acpThread.h"
#include "unix_acpRobotOSFactory.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpRobotOSFactory constructor 
 */

unix_acpRobotOSFactory::unix_acpRobotOSFactory() :
  acpRobotOSFactory()
{
} /* unix_acpRobotOSFactory constructor */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpRobotOSFactory destructor 
 */

unix_acpRobotOSFactory::~unix_acpRobotOSFactory()
{
} /* unix_acpRobotOSFactory destructor */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpRobotOSFactory buildMutex method
 */

acpMutex* unix_acpRobotOSFactory::buildMutex(const char* name)
{
  return new unix_acpMutex(name);
  
} /* unix_acpRobotOSFactory buildMutex method*/



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpRobotOSFactory buildThread method
 */

acpThread* unix_acpRobotOSFactory::buildThread(const char* name)
{
  return new unix_acpThread(name);
  
} /* unix_acpRobotOSFactory buildThread method*/
