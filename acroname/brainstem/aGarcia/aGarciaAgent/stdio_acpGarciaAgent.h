/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: stdio_acpGarciaAgent.h                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the GarciaAgent application          //
//              object.                                            //
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

#ifndef _stdio_aGarciaAgent_H_
#define _stdio_aGarciaAgent_H_

#include "acpGarciaAgent.h"




class stdio_acpGarciaAgent :
  public acpGarciaAgent {
  public:
                                stdio_acpGarciaAgent();
    virtual			~stdio_acpGarciaAgent();

    virtual void		run();

    virtual void		cmdExit();

    virtual void		enableButton(
				  const int nCmd,
				  const bool bEnable)
                                  {}

    virtual void		setStatusText(const char* msg);

    virtual void		updateHeartbeat();

  private:
    bool                        m_bDone;
};

#endif // _stdio_aGarciaAgent_H_
