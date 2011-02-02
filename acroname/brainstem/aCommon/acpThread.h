/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: acpThread.h	                                           */
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

#ifndef _acpThread_H_
#define _acpThread_H_

#include "aUtil.h"
#include "acpList.h"
#include "acpMessage.h"

#define aMAXTHREADSYNCWAITMS	2000

class acpRunable;

class aUTIL_EXPORT acpThread {

  public:
				acpThread(const char* name);
    virtual			~acpThread();

    virtual void		start(
    				  acpRunable* pRunable) = 0;

    virtual void		sendMessage(
    				  acpMessage* pMessage,
  				  const bool bAsync) = 0;

    virtual acpMessage*		getMessage() = 0;

    virtual bool		yield(
    				  const unsigned long msec) = 0;

    virtual bool		isDone() 
    				  { return m_bDone; }

    virtual bool		isThreadContext() = 0;

    virtual void		sync(
    				  acpMessage* pMessage) = 0;

  protected:
    bool			m_bRunning;
    bool			m_bDone;

    unsigned long		m_nThreadID;
    unsigned long		m_nCallerThreadID;

    acpRunable*			m_pRunable;

    acpList<acpMessage>		m_messages;
};

#endif /* _acpThread_H_ */

