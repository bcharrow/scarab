/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotCallbackMessage.h                                 //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the callback message object that     //
//              handles callbacks from the Robot thread to the     //
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

#ifndef _acpRobotCallbackMessage_H_
#define _acpRobotCallbackMessage_H_

#include "acpRobot.h"
#include "acpRobotInternal.h"
#include "acpMessage.h"
#include "acpRobotBehavior.h"

class acpRobotCallbackMessage :
  public acpMessage
{
  public:
	  		acpRobotCallbackMessage(
	  		  acpRobotInternal* pcRobot,
  			  acpRobotBehavior* pcBehavior,
  			  acpCallback* pcCallback,
  			  const bool bCompCB) :
  			m_pcRobot(pcRobot),
  			m_pcCallback(pcCallback),
  			m_pcBehavior(pcBehavior),
  			m_bCompCB(bCompCB) {}

   void	                process();

  private:
    acpRobotInternal*	m_pcRobot;
    acpCallback*	m_pcCallback;
    acpRobotBehavior*	m_pcBehavior;
    bool		m_bCompCB;
};

#endif // _acpRobotCallbackMessage_H_
