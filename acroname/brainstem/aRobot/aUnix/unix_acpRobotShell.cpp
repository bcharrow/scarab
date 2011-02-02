/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: unix_acpRobotShell.cpp                                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotShell client.              //
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

#include <ctype.h>
#include <sys/ioctl.h>

#ifdef TERMIOS
#include <termios.h>
#else /* TERMIOS */
#ifdef SGTTYB
#include <sgtty.h>
#else /* SGTTYB */
#include <termio.h>
#endif /* SGTTYB */
#endif /* !TERMIOS */

#ifdef TERMIOS
	struct termios oldtty, noecho;
#else
#ifdef SGTTYB
	struct sgttyb oldtty, noecho;
#else
	struct termio oldtty, noecho;
#endif
#endif

#include "aMemLeakDebug.h"
#include "acpException.h"
#include "unix_acpRobotShell.h"


/////////////////////////////////////////////////////////////////////

unix_acpRobotShell::unix_acpRobotShell() :
  acpRobotShell()
{
}


/////////////////////////////////////////////////////////////////////

unix_acpRobotShell::~unix_acpRobotShell() 
{
}


/////////////////////////////////////////////////////////////////////

bool unix_acpRobotShell::osGetInput() 
{	
  char ch;
  int i;
  
  //ignore leading whitespace
  while((ch = getchar()) == ' ' || ch == '\t');

  if( ch != '\n') {
    m_buffer[0] = ch;
    fgets(&m_buffer[1], INPUTBUFSIZE, stdin);
    strtok(m_buffer,"\n");
    i = strlen(m_buffer) - 1;

    while((i > 0) && isspace(m_buffer[i]))
      i--;
    if(i >= 0)
      m_buffer[i+1] = 0;  
  }
  return 1;
}


/////////////////////////////////////////////////////////////////////

int main(
  const int argc,
  const char* argv[]
) 
{
  int retVal = 1;

  try {
    unix_acpRobotShell app;
    app.init(argc, argv);
    retVal = app.run();
  } catch (const acpException& exception) {
    printf("Exception: %s\n", exception.msg());
  } catch (...) {
    printf("Unknown Exception\n");    
  }

  aLeakCheckCleanup();
}
