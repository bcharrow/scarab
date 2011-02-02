/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_acpGarciaOSFactory.cpp                               */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: The pure virtual class factory for the Garcia lib. */
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
#include "unix_acpGarciaOSFactory.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpGarciaOSFactory constructor 
 */

unix_acpGarciaOSFactory::unix_acpGarciaOSFactory() :
  acpGarciaOSFactory()
{
} /* unix_acpGarciaOSFactory constructor */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpGarciaOSFactory destructor 
 */

unix_acpGarciaOSFactory::~unix_acpGarciaOSFactory()
{
} /* unix_acpGarciaOSFactory destructor */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpGarciaOSFactory buildMutex method
 */

acpMutex* unix_acpGarciaOSFactory::buildMutex(const char* name)
{
  return new unix_acpMutex(name);
  
} /* unix_acpGarciaOSFactory buildMutex method*/



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * unix_acpGarciaOSFactory buildThread method
 */

acpThread* unix_acpGarciaOSFactory::buildThread(const char* name)
{
  return new unix_acpThread(name);
  
} /* unix_acpGarciaOSFactory buildThread method*/
