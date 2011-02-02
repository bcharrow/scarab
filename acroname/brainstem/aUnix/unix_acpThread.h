/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_acpThread.h                                          */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of windows thread class.                */
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

#ifdef aUNIX

#ifndef _unix_acpThread_H_
#define _unix_acpThread_H_

#ifdef aMACX
#include <Carbon/Carbon.h>
#endif /* aMACX */

#include <pthread.h>

#include "acpMessage.h"
#include "acpThread.h"

class unix_acpThread : public acpThread {

  public:
                        unix_acpThread(const char* name);
    virtual             ~unix_acpThread();
    
    void		start(acpRunable* pRunable);
    
    void		sendMessage(acpMessage* pMessage,
                                    const bool bAsync);
    acpMessage*		getMessage();

    virtual bool	yield(const unsigned long msec);

    virtual bool	isThreadContext();

    virtual void	sync(acpMessage* pMessage);

  private:
    int			runProc();
    static void*	threadProc(void* vpRef);

    bool                m_bInited;
    pthread_mutex_t	m_messageListMutex;
    pthread_mutex_t     m_yieldCondMutex;
    pthread_cond_t	m_yieldCond;
    pthread_mutex_t     m_syncCondMutex;
    pthread_cond_t	m_syncCond;

    pthread_t		m_thread;
};

#endif /* _unix_acpThread_H_ */

#endif /* UNIX */
