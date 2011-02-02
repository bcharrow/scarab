/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_acpThread.cpp                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Definition of unix thread class.                   */
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

#include "aAssert.h"
#include "unix_acpThread.h"
#include "acpException.h"

#include "acpRunable.h"

#include <sys/time.h>
#include <errno.h>


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

unix_acpThread::unix_acpThread(const char* name) :
  acpThread(name),
  m_bInited(false),
  m_thread((pthread_t)NULL)
{
  pthread_mutex_init(&m_messageListMutex, NULL);

  pthread_mutex_init(&m_syncCondMutex, NULL);
  pthread_cond_init(&m_syncCond, NULL);

  pthread_mutex_init(&m_yieldCondMutex, NULL);
  pthread_cond_init(&m_yieldCond, NULL);

  m_bInited = true;
  
} // unix_acpThread constructor



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

unix_acpThread::~unix_acpThread()
{
  if (m_bRunning && m_thread) {
    m_bDone = true;
    pthread_join(m_thread, NULL);
  }

  pthread_mutex_destroy(&m_messageListMutex);

  pthread_cond_destroy(&m_yieldCond);
  pthread_mutex_destroy(&m_yieldCondMutex);

  pthread_cond_destroy(&m_syncCond);
  pthread_mutex_destroy(&m_syncCondMutex);

} // unix_acpThread destructor



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void unix_acpThread::start(acpRunable* pRunable)
{
  m_nCallerThreadID = (unsigned long)pthread_self();

  m_pRunable = pRunable;

  pthread_create(&m_thread, NULL, threadProc, this);

} // unix_acpThread start method



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sendMessage
 *
 * this code can be called from anywhere
 */

void unix_acpThread::sendMessage(acpMessage* pMessage,
				 const bool bAsync)
{
  // shouldn't message ourselves
  aAssert(!isThreadContext());

  pthread_mutex_lock(&m_messageListMutex);
  
  // create a condition to wait on if we are synchronous
  if (!bAsync) {
    pMessage->m_synchronize = (void*)&m_syncCond;
    pMessage->m_pcThread = this;
  }

  m_messages.addToHead(pMessage);

  pthread_mutex_unlock(&m_messageListMutex);

  // signal this thread if it is yielding since there is a message
  pthread_mutex_lock(&m_yieldCondMutex);
  pthread_cond_signal(&m_yieldCond);
  pthread_mutex_unlock(&m_yieldCondMutex);

  if (!bAsync) {
    // lock the mutex while we set up the yield
    pthread_mutex_lock(&m_syncCondMutex);

    // compute the requested timeout
    struct timeval now;
    struct timespec timeout;
    gettimeofday(&now, NULL);
    int secs = aMAXTHREADSYNCWAITMS / 1000;
    int mod = aMAXTHREADSYNCWAITMS % 1000;
    unsigned long nsec = (now.tv_usec * 1000) + (mod * 1000000);
    if (nsec > 1000000000) {
      secs += nsec / 1000000000;
      nsec %= 1000000000;
    }
    timeout.tv_sec = now.tv_sec + secs;
    timeout.tv_nsec = nsec; 
    pthread_cond_timedwait(&m_syncCond, &m_syncCondMutex, &timeout);

    pthread_mutex_unlock(&m_syncCondMutex);
  }

} // unix_acpThread sendMessage method



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * getMessage
 * 
 * can be called from anywhere
 */

acpMessage* unix_acpThread::getMessage()
{
  pthread_mutex_lock(&m_messageListMutex);

  acpMessage* pMessage = m_messages.removeTail();

  pthread_mutex_unlock(&m_messageListMutex);

  return pMessage;

} // unix_acpThread getMessage method



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * yield method
 *
 * This can only be called from this thread's context.
 *
 * Yields the thread for the requested time and returns true if it
 * the entire interval passed without any new messages coming in.
 */

bool unix_acpThread::yield(const unsigned long msec)
{
  /* ensure we are in this thread's context */
  aAssert(pthread_equal(m_thread, pthread_self()));

  // make sure we are initialized
  aAssert(m_bInited);

  // this is a fringe case where we do nothing
  if (!msec)
    return false;

  bool bDidYield = true;

  // don't block if there are messages available
  pthread_mutex_lock(&m_messageListMutex);
  if (m_messages.length())
    bDidYield = false;
  pthread_mutex_unlock(&m_messageListMutex);

  // attempt to yield, waiting for either a timeout or a new message
  // signal
  if (bDidYield) {

    // lock the mutex while we set up the yield
    pthread_mutex_lock(&m_yieldCondMutex);

    // compute the requested timeout
    struct timeval now;
    struct timespec timeout;
    gettimeofday(&now, NULL);
    int secs = msec / 1000;
    int mod = msec % 1000;
    unsigned long nsec = (now.tv_usec * 1000) + (mod * 1000000);
    if (nsec > 1000000000) {
      secs += nsec / 1000000000;
      nsec %= 1000000000;
    }
    timeout.tv_sec = now.tv_sec + secs;
    timeout.tv_nsec = nsec; 
    int result = pthread_cond_timedwait(&m_yieldCond, 
					&m_yieldCondMutex, &timeout);

    // we don't need the mutex now
    pthread_mutex_unlock(&m_yieldCondMutex);

    bDidYield = (result == ETIMEDOUT);
  }

  return bDidYield;

} // unix_acpThread yield method



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * isThreadContext
 */

inline bool unix_acpThread::isThreadContext()
{
  return pthread_equal(m_thread, pthread_self());

} // unix_acpThread isThreadContext method



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sync
 */

void unix_acpThread::sync(acpMessage* pMessage)
{
  aAssert(pMessage);
  aAssert(pMessage->m_synchronize);

  pthread_cond_signal((pthread_cond_t*)pMessage->m_synchronize);

} // unix_acpThread sync method



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * runProc
 */

int unix_acpThread::runProc()
{
  m_thread = pthread_self();
  m_nThreadID = (unsigned long)m_thread;
  m_bRunning = true;

  int retVal;
  
  try {
    retVal = m_pRunable->run();
  } catch (acpException& exception) {
    retVal = -1;
  }

  m_bRunning = false;

  return(retVal);

} /* unix_acpThread runProc method */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void* unix_acpThread::threadProc(void* vpRef)
{
  unix_acpThread* pThread = (unix_acpThread*)vpRef;
  pThread->runProc();

  /* be sure the done flag is set when the proc is done */
  pThread->m_bDone = true;

  return(NULL);

} /* unix_acpThread threadProc */


#endif /* aUNIX */
