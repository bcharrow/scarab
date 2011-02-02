//                                                                 //
//                                                                 //
// file: acpRobotShell.h                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotShell client.              //
//                                                                 //
/////////////////////////////////////////////////////////////////////
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

#ifndef _acpRobotShell_H_
#define _acpRobotShell_H_

#include "acpList.h"
#include "acpRobotClient.h"
#include "acpRobotObject.h"

#define INPUTBUFSIZE             1024
#define aROBOTSHELL_TIMEOUT      500
#define aROBOTSHELL_DEFAULTPORT  8008

class acpRobotShell :
  public acpRobotClient
{
  public:
  				acpRobotShell();
    virtual			~acpRobotShell();

    void			init(
    				  const int argc,
    				  const char* argv[]);

    virtual int			run();

    virtual void		handleInput();
    
    virtual bool		osGetInput() = 0;
    
  protected:
    bool			m_bDone;

    acpList<acpRobotObject>	m_stack;
    acpRobotObject* 		m_pCurrent;

    // utility routines
    void			usageExit(
    				  const char* pMsg = NULL);

    // command line stuff
    bool			m_bInput;
    char			m_buffer[INPUTBUFSIZE];
    aTokenizerRef		m_tokenizer;
    void			prompt();
    void			badInput(
    				  const char* pMsg = NULL);

    // command list stuff
    typedef int (*cmdProc)(acpRobotShell& shell);
    typedef struct cmdEntry {
      const char* 		cmdName;
      cmdProc			cmdHandler;
    } cmdEntry;
    static cmdEntry		cmdList[];

    static int			cmdHelp(acpRobotShell& shell);
    static int			cmdExit(acpRobotShell& shell);
    static int			cmdProperties(acpRobotShell& shell);
    static int			cmdObjects(acpRobotShell& shell);
    static int			cmdSet(acpRobotShell& shell);
    static int			cmdGet(acpRobotShell& shell);
    static int			cmdRGet(acpRobotShell& shell);
    static int			cmdChangeObject(acpRobotShell& shell);
    static int			cmdDocument(acpRobotShell& shell);
    
    friend class		acpRobotObject;
    friend class		acpRobotProperty;

  private:
    void			documentObjectHTML(
     				  acpRobotObject* pObject);
};


#endif // _acpRobotShell_H_
