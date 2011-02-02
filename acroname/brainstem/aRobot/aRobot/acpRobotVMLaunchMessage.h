/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotVMLaunchMessage.h                                 //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the VM Launch message sent to the    //
//              private thread in the Robot API library object.    //
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

#ifndef _acpRobotVMLaunchMessage_H_
#define _acpRobotVMLaunchMessage_H_

#include "acpRobotInternal.h"

class acpRobotVMLaunchMessage :
  public acpMessage
{
  public:

  			acpRobotVMLaunchMessage(
			  acpRobotInternal* pRobot,
			  const int nLinkID,
			  const int nPID,
			  char* pCup,
			  const unsigned long ulSize,
			  char* pcInputBuff,
			  aTEAVMExitProc exitProc,
			  void* pExitRef) :
  			    m_pcRobotInternal(pRobot),
  			    m_nLinkID(nLinkID),
  			    m_nPID(nPID),
			    m_pCUP(pCup),
			    m_ulSize(ulSize),
			    m_pInputData(pcInputBuff),
  			    m_exitProc(exitProc),
  			    m_pExitRef(pExitRef) {}

    void		process();


  private:

    acpRobotInternal*	m_pcRobotInternal;

    int			m_nLinkID;
    int			m_nPID;
    char*		m_pCUP;
    unsigned long	m_ulSize;
    char*		m_pInputData;
    aTEAVMExitProc	m_exitProc;
    void*		m_pExitRef;
};


class acpRobotVMKillMessage :
  public acpMessage
{
  public:

  			acpRobotVMKillMessage(
			  acpRobotInternal* pRobot,
			  const int nPID) :
  			    m_pcRobotInternal(pRobot),
  			    m_nPID(nPID) {}

    void		process();


  private:

    acpRobotInternal*	m_pcRobotInternal;
    int			m_nPID;
};

#endif // _acpRobotVMLaunchMessage_H_
