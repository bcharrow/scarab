/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpCallbackMessage.h                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the CallbackMessage object that      //
//              handles callbacks from the garcia thread to the    //
//              caller.                                            //
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

#ifndef _acpCallbackMessage_H_
#define _acpCallbackMessage_H_

#include "acpGarcia.h"
#include "acpGarciaInternal.h"
#include "acpMessage.h"
#include "acpBehavior.h"

class acpCallbackMessage :
  public acpMessage
{
  public:
	  		acpCallbackMessage(
	  		  acpGarciaInternal* pcGarcia,
  			  acpBehavior* pcBehavior,
  			  acpCallback* pcCallback) :
  			m_pcGarcia(pcGarcia),
  			m_pcCallback(pcCallback),
  			m_pcBehavior(pcBehavior) {}

   void	                process();

  private:
    acpGarciaInternal*	m_pcGarcia;
    acpCallback*	m_pcCallback;
    acpBehavior*	m_pcBehavior;
};

#endif // _acpCallbackMessage_H_
