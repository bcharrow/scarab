/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotShell.cpp                                         //
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

#include "aUtil.h"
#include "aVersion.h"
#include "aStream_TextLine.h"
#include "acpException.h"
#include "acpRobotShell.h"
#include "acpHTMLDocument.h"
#include "acpProperty.h"


acpRobotShell::cmdEntry acpRobotShell::cmdList[] = {
  {"help", 		acpRobotShell::cmdHelp}, 
  {"exit", 		acpRobotShell::cmdExit}, 
  {"properties", 	acpRobotShell::cmdProperties}, 
  {"objects",	 	acpRobotShell::cmdObjects}, 
  {"set",	 	acpRobotShell::cmdSet}, 
  {"get",	 	acpRobotShell::cmdGet}, 
  {"rget",	 	acpRobotShell::cmdRGet}, 
  {"co",	 	acpRobotShell::cmdChangeObject}, 
  {"document",	 	acpRobotShell::cmdDocument}, 
  {NULL,NULL}
};

 
/////////////////////////////////////////////////////////////////////

acpRobotShell::acpRobotShell() :
  acpRobotClient(),
  m_bDone(false),
  m_pCurrent(NULL),
  m_bInput(false),
  m_tokenizer(NULL)
{
  m_buffer[0] = 0;
}


/////////////////////////////////////////////////////////////////////

acpRobotShell::~acpRobotShell() 
{
}


/////////////////////////////////////////////////////////////////////

void acpRobotShell::init(
  const int argc,
  const char* argv[]
) 
{
  printf("Acroname Robot Shell\n");
  printf("Version %i.%i, ", (int)aVERSION_MAJOR, (int)aVERSION_MINOR);
  printf("Build %i\n", (int)aROBOT_BUILD_NUM);
  printf("Copyright 1994-2005 (c) Acroname Inc.\n");

  // get our IP address if not provided
  aErr e = aErrNone;
  const char* p;

  unsigned long ipaddress;
  unsigned short ipport = aROBOTSHELL_DEFAULTPORT;
  if (argc < 2) {
    if (aIO_GetInetAddr(m_ioRef, &ipaddress, &e))
      throw acpException(e, "unable to obtain an IP address");
 
  // parse the inet address provided
  // (bit of a hack to handle last value)
  } else {
    e = aULong_FromInetAddr(&ipaddress, argv[1]);
    if (e != aErrNone)
      usageExit("invalid IP address");
  }

  // optional argument
  if (argc > 2) {
    p = argv[2];
    int port;
    aIntFromString(&port, p);
    ipport = (unsigned short)port;
  }

  // establish the connection to the robot agent
  int id = connect(ipaddress, ipport, aROBOTSHELL_TIMEOUT);
  if (id) {
    acpString name;
    getObjectName(&name, id);
    m_pCurrent = new acpRobotObject(this, "root", (char*)name);
    m_pCurrent->setID(id);
    m_stack.add(m_pCurrent);
  }
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::run() 
{
  prompt();
  while (!m_bDone) {
    m_bInput = osGetInput();
    if (m_bInput) {
      handleInput();
      if (!m_bDone) {
        prompt();
      }
    }
  }
  return 0;
}


/////////////////////////////////////////////////////////////////////

void acpRobotShell::handleInput() 
{
  if (strlen(m_buffer)) {
    aStreamRef stream;
    aErr e;
    e = aStream_Create_TextLine_Input(m_ioRef, 
    				      m_buffer, 
    				      &stream);

    if (e == aErrNone)
      aTokenizer_Create(m_ioRef, 
    		        stream, 
    		        "command line",
    		        aFileAreaUser,
    		        NULL,
    		        NULL, 
    		        &m_tokenizer,
    		        &e);

    // inspect the first token to see if it is a command
    aBool bAvail = aFalse;
    aToken* pToken;
    if (e == aErrNone)
      bAvail = !aTokenizer_Next(m_ioRef, m_tokenizer, &pToken, NULL);

    if (bAvail == aTrue) {
      switch (pToken->eType) {

      case tkIdentifier: 
        {
	  cmdEntry* entry = cmdList;
          while (entry->cmdName) {
            // see if the N characters in token name match first N 
	    // characters in command name (makes it possible to 
	    // abbreviate commands, like prop for properties)
            if (!strncmp(entry->cmdName, pToken->v.identifier, 
			 aStringLen(pToken->v.identifier))) {
              entry->cmdHandler(*this);
              m_buffer[0] = 0;
              break;
            }
            entry++;
          }
          if (!entry->cmdName)
            badInput();
        }
        break;

      default:
        badInput();
        break;

      } // switch
    }
   
    // clean up
    aTokenizer_Dispose(m_ioRef, m_tokenizer, pToken, &e);
    aTokenizer_Destroy(m_ioRef, m_tokenizer, NULL);
    m_tokenizer = NULL;
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotShell::usageExit(
  const char* pMsg) 
{
  if (pMsg)
    printf("parameter error: %s\n", pMsg);
  printf("usage: RobotShell addr [port]\n");
  printf(" for example\n");
  printf("   RobotShell 127.0.0.1 \n");
  printf(" or\n");
  printf("   RobotShell 127.0.0.1 8000\n");
  exit(1);
}


/////////////////////////////////////////////////////////////////////
// displays the current prompt

void acpRobotShell::prompt() 
{
  int i = 0;
  aLISTITERATE(acpRobotObject, m_stack, pObject) {
    if (i++ != 0)
      printf("/");
    printf(pObject->getName());
  }
  printf("> ");
  fflush(stdout);
}


/////////////////////////////////////////////////////////////////////
// displays bad input message

void acpRobotShell::badInput(
  const char* pMsg) 
{
  if (pMsg)
    printf("%s\n", pMsg);
  else
    printf("unrecognized input: %s\n", m_buffer);
  fflush(stdout);
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdHelp(
  acpRobotShell& shell) 
{
  int k = 0;
  while (cmdList[k].cmdName) {
    printf("%s\n", cmdList[k].cmdName);
    k++;
  }
  return 0;
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdExit(
  acpRobotShell& shell) 
{
  shell.m_bDone = true;
  return 0;
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdProperties(
  acpRobotShell& shell) 
{
  if (shell.m_bConnected) {

    aErr e;
    aToken* pToken;
    bool bVerbose = false;

    shell.m_pCurrent->ensurePropertiesEnumerated();

    if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
      switch (pToken->eType) {

      case tkIdentifier: 
        {
          // check for more options
          if (!aStringCompare(pToken->v.identifier, "debug")) {
            bVerbose = true;
          }
        }
        break;
      }

      // clean up
      aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
    }

    aLISTITERATE(acpRobotProperty, 
  	         shell.m_pCurrent->m_properties, pProperty) {
      if (bVerbose) {
        printf("%4i ID=%08X %08X %s\n",
	       pProperty->getIndex(),
	       pProperty->getOwner()->getID(),
	       pProperty->getTypeFlags(),
	       pProperty->getName());
        printf("  %s\n", pProperty->getDescription());
      } else {
        printf("%s\n", pProperty->getName());
      }
    }
  }

  return 0;
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdObjects(
  acpRobotShell& shell) 
{
  if (shell.m_bConnected) {

    aErr e;
    aToken* pToken;
    bool bVerbose = false;

    shell.m_pCurrent->ensureObjectsEnumerated();

    if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
      switch (pToken->eType) {

      case tkIdentifier: 
        {
          // check for more options
          if (!aStringCompare(pToken->v.identifier, "debug")) {
            bVerbose = true;
          }
        }
        break;
      }

      // clean up
      aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
    }

    aLISTITERATE(acpRobotObject, 
  	         shell.m_pCurrent->m_children, pObject) {
      if (bVerbose) {
        printf("Parent=%08X ID=%08X %s\n",
	       (int)(long)pObject->getParent(),
	       pObject->getID(),
	       pObject->getName());
      } else {
        printf("%s\n", pObject->getName());
      }
    }
  }

  return 0;
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdSet(
  acpRobotShell& shell) 
{
  if (shell.m_bConnected) {
    aErr e;
    aToken* pToken;

    if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
      switch (pToken->eType) {

      case tkIdentifier: 
        {
          // find the property
          acpRobotProperty* pProperty = 
            shell.m_pCurrent->getProperty(pToken->v.identifier);
          if (pProperty) {
            pProperty->set();
          } else {
            acpString msg("no property named: ");
            msg += pToken->v.identifier;
            shell.badInput(msg);
          }
        }
        break;

      case tkString: 
        {
          // find the property
          acpRobotProperty* pProperty = 
            shell.m_pCurrent->getProperty(pToken->v.string);
          if (pProperty) {
            pProperty->set();
          } else {
            acpString msg("no property named: ");
            msg += pToken->v.string;
            shell.badInput(msg);
          }
        }
        break;

      default:
        shell.badInput();
        break;

      } // switch

      // clean up
      aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
    }
  }
  
  return 0;
}



/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdGet(
  acpRobotShell& shell) 
{
  if (shell.m_bConnected) {
    aErr e;
    aToken* pToken;

    if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
      switch (pToken->eType) {

      case tkIdentifier: 
        {
          // find the property
          acpRobotProperty* pProperty = 
            shell.m_pCurrent->getProperty(pToken->v.identifier);
          if (pProperty) {
            pProperty->get();
          } else {
            acpString msg("no property named: ");
            msg += pToken->v.identifier;
            shell.badInput(msg);
          }
        }
        break;

      case tkString: 
        {
          // find the property
          acpRobotProperty* pProperty = 
            shell.m_pCurrent->getProperty(pToken->v.string);
          if (pProperty) {
            pProperty->get();
          } else {
            acpString msg("no property named: ");
            msg += pToken->v.string;
            shell.badInput(msg);
          }
        }
        break;

      default:
        shell.badInput();
        break;

      } // switch

      // clean up
      aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
    }
  }
  
  return 0;
}



/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdRGet(
  acpRobotShell& shell) 
{
  if (shell.m_bConnected) {
    aErr e;
    aToken* pToken;
    acpRobotProperty* pProperty = NULL;
    int nReps = 0;

    if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
      switch (pToken->eType) {

      case tkIdentifier: 
        {
          // find the property
          pProperty = 
            shell.m_pCurrent->getProperty(pToken->v.identifier);
          if (!pProperty) {
            acpString msg("no property named: ");
            msg += pToken->v.identifier;
            shell.badInput(msg);
          }
        }
        break;

      case tkString: 
        {
          // find the property
          pProperty = 
            shell.m_pCurrent->getProperty(pToken->v.string);
          if (!pProperty) {
            acpString msg("no property named: ");
            msg += pToken->v.string;
            shell.badInput(msg);
          }
        }
        break;

      default:
        shell.badInput();
        break;

      } // switch

      // clean up
      aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
    }


    if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
      switch (pToken->eType) {

      case tkInt: 
        {
          // find rep count
          nReps = pToken->v.integer;
          if (nReps > 200) nReps = 200;
          if (nReps < 1) nReps = 1;
          printf("%i reps\n", nReps);

          unsigned long tStart;
          unsigned long tStop;
          unsigned long tElapsed;
          float tAvg;
          aIO_GetMSTicks(shell.m_ioRef, &tStart, NULL);
          for (int i = 0; i < nReps; i++) {
            if (pProperty)
              pProperty->get();
          }
          
          // time diagnostics
          aIO_GetMSTicks(shell.m_ioRef, &tStop, NULL);
          tElapsed = tStop - tStart;
          tAvg = ((float)tElapsed)/((float)nReps);
          printf("total time = %ims, %.4fms/op\n", (int)tElapsed, tAvg);
        }
        break;

      default:
        shell.badInput();
        break;

      } // switch

      // clean up
      aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
    }

  }
  
  return 0;
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdChangeObject(
  acpRobotShell& shell) 
{
  if (shell.m_bConnected) {

    aErr e;
    aToken* pToken;

    shell.m_pCurrent->ensureObjectsEnumerated();

    if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
      switch (pToken->eType) {

      case tkSpecial:
        {
          // catches .. (parent object) argument
          acpRobotObject* pObject = NULL;
          if (pToken->v.identifier[0] == '.') {
            pObject = shell.m_pCurrent->getParent();
          }
          if (pObject) {
            shell.m_pCurrent = pObject;
            printf("changed to object:  %s\n", pObject->getName());
          } else {
            acpString msg("no parent object");
            shell.badInput(msg);
          }
        }
        break;

      case tkIdentifier: 
        {
          acpRobotObject* pObject =
            shell.m_pCurrent->getObject(pToken->v.identifier);
          if (pObject) {
            shell.m_pCurrent = pObject;
            printf("changed to object:  %s\n", pObject->getName());
          } else {
            acpString msg("no ojbect named: ");
            msg += pToken->v.identifier;
            shell.badInput(msg);
          }
        }
        break;

      case tkString: 
        {
          // find the object
          acpRobotObject* pObject = 
            shell.m_pCurrent->getObject(pToken->v.string);
          if (pObject) {
            shell.m_pCurrent = pObject;
            printf("changed to object:  %s\n", pObject->getName());
          } else {
            acpString msg("no object named: ");
            msg += pToken->v.string;
            shell.badInput(msg);
          }
        }
        break;

      default:
        shell.badInput();
        break;

      } // switch

      // clean up
      aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
    }
    
  }

  return 0;
}


/////////////////////////////////////////////////////////////////////

int acpRobotShell::cmdDocument(
  acpRobotShell& shell) 
{
  aErr e;
  aToken* pToken;

  shell.m_pCurrent->ensureObjectsEnumerated();
  shell.m_pCurrent->ensurePropertiesEnumerated();

  if (!aTokenizer_Next(shell.m_ioRef, shell.m_tokenizer, 
  		       &pToken, &e)) {
    switch (pToken->eType) {

    case tkIdentifier:

      // document in html
      if (!aStringCompare(pToken->v.identifier, "html")) {
	shell.documentObjectHTML(shell.m_pCurrent);
      } else
	shell.badInput();
      break;

    default:
      shell.badInput();
      break;

    } // switch

    // clean up
    aTokenizer_Dispose(shell.m_ioRef, shell.m_tokenizer, pToken, &e);
  }

  return 0;
}


/////////////////////////////////////////////////////////////////////

void acpRobotShell::documentObjectHTML(
  acpRobotObject* pObject) 
{
  printf("documenting %s in html format\n", pObject->getName());
  acpString filename(pObject->getName());
  filename += "_properties.html";
  acpHTMLDocument doc(m_ioRef, filename, aFileAreaAsset);

  doc += "<HEAD>";
  doc += "<TITLE>";
  doc += pObject->getName();
  doc += "</TITLE>";
  doc += "</HEAD>";

  doc += "<BODY>";

  doc += "<H2>";
  doc += pObject->getName();
  doc += " Object"; 
  doc += "</H2>";

  doc += "<TABLE BORDER=1 WIDTH='100%' CELLPADDING=0"
		" CELLSPACING=0>";

  doc += "<TR>";
  doc += "<TH>property</TH>";
  doc += "<TH>type</TH>";
  doc += "<TH>description</TH>";
  doc += "</TR>";

  aLISTITERATE(acpRobotProperty, pObject->m_properties, pProperty) {
    doc += "<TR VALIGN=TOP>";

    doc += "<TD>";
    doc += pProperty->getName();
    doc += "</TD>";
  
    int flags = pProperty->getTypeFlags();
    int type = flags & aPROPERTY_TYPE_MASK; 

    doc += "<TD NOWRAP>";
    switch (type) {
    case aPROPERTY_FLAG_INT:
      doc += "int";
      break;
    case aPROPERTY_FLAG_FLOAT:
      doc += "float";
      break;
    case aPROPERTY_FLAG_STRING:
      doc += "string";
      break;
    case aPROPERTY_FLAG_BOOL:
      doc += "boolean";
      break;
    case aPROPERTY_FLAG_CALLBACK:
      doc += "callback";
      break;
    case aPROPERTY_FLAG_VOIDPTR:
      doc += "void pointer";
      break;
    case aPROPERTY_FLAG_OBJECT:
      doc += "object";
      break;
    } // switch

    // add in permission information	  
    if ((flags & aPROPERTY_FLAG_READ) 
        && (flags & aPROPERTY_FLAG_WRITE))
      doc += "(readable and writable)";
    else if (flags & aPROPERTY_FLAG_READ)
      doc += "(readable)";
    else if (flags & aPROPERTY_FLAG_WRITE)
      doc += "(writeable)";

    doc += "</TD>";

    doc += "<TD>";
    doc += pProperty->getDescription();
    doc += "</TD>";
  
    doc += "</TR>";
  }
  doc += "</TABLE>";
  doc += "</BODY>";
}
