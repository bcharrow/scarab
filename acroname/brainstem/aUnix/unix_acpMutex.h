/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_acpMutex.h	                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of a platform-independent thread        */
/*              construction.  This is an abstract class that      */
/*              is implemented on different platforms by           */
/*              sub-classing this class.                           */
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

#ifndef _unix_acpMutex_H_
#define _unix_acpMutex_H_

#include <pthread.h>

#include "acpMutex.h"

class unix_acpMutex :
  public acpMutex
{
  public:
  			unix_acpMutex(const char* name);
    virtual		~unix_acpMutex();

    virtual void	lock();
    virtual void	unlock();
 
  private:
#ifdef aDEBUG
  bool			m_bLocked;
#endif
    pthread_mutex_t     m_mutex;
};

#endif /* _unix_acpMutex_H_ */
