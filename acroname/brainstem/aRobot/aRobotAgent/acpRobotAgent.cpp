/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotAgent.cpp                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the RobotAgent application       //
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

#include <math.h>

#include "acpException.h"
#include "acpRobotAgent.h"
#include "aUtil.h"



/////////////////////////////////////////////////////////////////////

acpRobotAgent::acpRobotAgent() :
  m_name("aRobot"),
  m_bFinalized(false),
  m_bHB(false),
  m_nTimerSetting(100),
  m_pRobot(NULL),
  m_batteryCheck(0),
  m_lastStatus(0),
  m_logView(NULL),
  m_nPort(aROBOTAGENT_DEFAULTPORT),
  m_socket(NULL),
  m_buffer(NULL),
  m_linkUpInterval(0),
  m_eMode(kInit),
  m_bConnected(false),
  m_nUsers(0),
  m_nExecuteCBIndex(-1),
  m_nCompletionCBIndex(-1),
  m_nCompletionStatusIndex(-1)
{
  aErr err;
  if (aIO_GetLibRef(&m_ioRef, &err))
    die("unable to open aIO library");
  
  if (aUI_GetLibRef(&m_uiRef, &err))
    die("unable to open aUI library");

  if (aSettingFile_Create(m_ioRef, 
  			  aROBOTAGENT_MAXSETTINGLEN, 
  			  aROBOTAGENT_SETTINGSFILE,
			  &m_settings, &err))
    die("unable to create settings file");

  aIO_GetInetAddr(m_ioRef, &m_nInetAddr, &err);
  aAssert(err == aErrNone);

  if (aStreamBuffer_Create(m_ioRef, 1024, &m_buffer, NULL))
    die("unable to create output buffer");

} // acpRobotAgent constructor



/////////////////////////////////////////////////////////////////////

acpRobotAgent::~acpRobotAgent()
{
  if (m_buffer) {
    aStream_Destroy(m_ioRef, m_buffer, NULL);
    m_buffer = NULL;
  }
  if (m_socket) {
    aStream_Destroy(m_ioRef, m_socket, NULL);
    m_socket = NULL;
  }
  if (m_logView) {
    aStream_Destroy(m_uiRef, m_logView, NULL);
    m_logView = NULL;
  }
  
  if (m_settings)
    aSettingFile_Destroy(m_ioRef, m_settings, NULL);
  
  if (m_uiRef) {
    aUI_ReleaseLibRef(m_uiRef, NULL);
    m_uiRef = NULL;
  }

  if (m_ioRef) {
    aIO_ReleaseLibRef(m_ioRef, NULL);
    m_ioRef = NULL;
  }

  if (m_pRobot)
    delete m_pRobot;

} // acpRobotAgent destructor


/////////////////////////////////////////////////////////////////////

bool acpRobotAgent::init(
  const int argc,
  const char* argv[])
{
  // get the name from the parameters
  for (int i = 1; i < argc; i++) {
    printf("argv[%d] = %s\n", i, argv[i]);
  }
  
  // select api name
  char* pName = NULL;
  aErr err;
  if (aSettingFile_GetString(m_ioRef, m_settings, 
  			     aROBOTAGENT_APINAMESETTING,
  		             &pName, (char*)m_name, &err))
    die("getting api file setting");

  if (m_pRobot)
    throw acpException(aErrConfiguration, 
			   "acpRobotAgent::init called multiple times");

  // build the actual aRobot api
  m_pRobot = new acpRobot(pName);

  // install a heartbeat callback object
  acpValue hbVal(new acpRobotAgentHB(this));
  m_pRobot->setNamedValue("heartbeat_callback", &hbVal);
  
  return true;
}


/////////////////////////////////////////////////////////////////////
// behavior completion callback

aErr acpRobotAgent::taskComplete (
  acpRobot* pRobot,
  acpObject* pBehavior
)
{
  char line[100];
  char num[10];

  acpRobotAgent* pAgent = 
    (acpRobotAgent*)pRobot->getNamedValue("app-object")->getVoidPtrVal();

  if (!pAgent)
    return aErrUnknown;

  pAgent->m_lastStatus = 
    pBehavior->getNamedValue("completion_status")->getIntVal();

  // show some status
  aStringCopy(line, "Agent completed with status ");
  aStringFromInt(num, pAgent->m_lastStatus);
  pAgent->addLogLine(line);

  return aErrNone;

} // acpRobotAgent taskComplete method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::resetSocket()
{
  printf("acpRobotAgent::resetSocket\n");

  // start fresh with no users
  m_nUsers = 0;

  // we don't have a client so we don't have a link timeout
  m_linkUpInterval = 0;

  // clean out an old socket if present
  if (m_socket) {
    aStream_Destroy(m_ioRef, m_socket, NULL);
    m_socket = NULL;
    m_bConnected = false;
  }

#ifdef aDEBUG
  char msg[100];
  aUtil_FormatInetAddr(msg, m_nInetAddr, NULL);
  printf("attemting to create socket at %s:%d\n", msg, m_nPort);
#endif // aDEBUG

  // build a new server socket for communication
  aErr err;
  aStream_CreateSocket(m_ioRef, 
    		       m_nInetAddr,
    		       m_nPort,
    		       aTrue,
    		       &m_socket,
    		       &err);
  aAssert(err == aErrNone);

  addLogLine("agent reset");

} // acpRobotAgent resetSocket method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::handleRequest (
  char cFirstChar
)
{
  switch (cFirstChar) {

  case 'A':
    doAck();
    break;

  case 'B':
    doBehavior();
    break;

  case 'C':
    doCallbacks();
    break;

  case 'D':
    doDescription();
    break;

  case 'L':
    doLinkUp();
    break;

  case 'N':
    doName();
    break;

  case 'O':
    doSubObjects();
    break;

  case 'P':
    doProperties();
    break;

  case 'R':
    doRead();
    break;

  case 'T':
    doTemp();
    break;

  case 'W':
    doWrite();
    break;

  case 'X':
    addLogLine("disconnecting");
    m_bConnected = false;
    resetSocket();
    break;

  default:
    char line[100];
    sprintf(line, "unknown command %d, %c", 
    	    cFirstChar, cFirstChar);
    addLogLine(line);
    break;

  } // switch

} // handleRequest


/////////////////////////////////////////////////////////////////////

void acpRobotAgent::respond (
  const char* pResponse
)
{
  if (m_socket) {
    unsigned int len = aStringLen(pResponse); 
    aStream_Write(m_ioRef, m_socket, pResponse, len, NULL);
  }

} // respond method



/////////////////////////////////////////////////////////////////////

aErr acpRobotAgent::propertyEnum (
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS typeFlags,
  void* vpRef
)
{
  acpRobotAgent* pAgent = (acpRobotAgent*)vpRef;

  if (!aStringCompare(pName, "execute_callback"))
    pAgent->m_nExecuteCBIndex = nIndex;
  else if (!aStringCompare(pName, "completion_callback"))
    pAgent->m_nCompletionCBIndex = nIndex;
  else if (!aStringCompare(pName, "completion_status"))
    pAgent->m_nCompletionStatusIndex = nIndex;

  pAgent->writeString(pName);
  pAgent->writeInt(typeFlags);

  return aErrNone;

} // propertyEnum method



/////////////////////////////////////////////////////////////////////

aErr acpRobotAgent::subObjectEnum (
  acpObject& object,
  void* vpRef
)
{
  acpRobotAgent* pAgent = (acpRobotAgent*)vpRef;

  // for each sub-object, write it's class and name
  // for each sub-object, write its address

  pAgent->writeString(object.getNamedValue("classname")->getStringVal());
  pAgent->writeString(object.getNamedValue("name")->getStringVal());
  pAgent->writeInt((int)(long)&object);

  return aErrNone;

} // subObjectEnum method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doAck()
{
  addLogLine("connection started");

  m_bConnected = true;

  if (!m_nUsers) {
    writeChar('A');
//    printf("root object is %X\n", (int)m_pRobot);
    writeObject(m_pRobot);
    m_nUsers = 1;
  } else {
//    printf("second login when busy\n");
    writeChar('B');
  }

  flush();

} // doAck method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doSync()
{
  // properties
  writeShort((short)m_pRobot->numProperties());
  m_pRobot->enumProperties(propertyEnum, this);

  // sub-objects
  writeShort((short)m_pRobot->numSubObjects());
  m_pRobot->enumSubObjects(subObjectEnum, this);

  // flush does the entire sync in a single (or minimal number of)
  // TCP packets
  flush();

} // doSync method


/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doName()
{
  acpObject* pObject = readObject();

  acpValue* pValue = pObject->getNamedValue("name");
  writeString(pValue->getStringVal());
  flush();

} // doName method


/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doSubObjects()
{
  acpObject* pObject = readObject();
  
  writeShort((aShort)pObject->numSubObjects());
  pObject->enumSubObjects(subObjectEnum, this);

  flush();

} // doSubObjects method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doProperties()
{
  acpObject* pObject = readObject();

#ifdef aDEBUG
  printf("enumerating object %X\n", (int)(long)pObject);
  printf("object has %d properties\n", pObject->numProperties());
#endif // aDEBUG

  writeShort((aShort)pObject->numProperties());
  pObject->enumProperties(propertyEnum, this);

  flush();

} // doProperties method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doBehavior()
{
  // get the primitive to perform
  short primitiveIndex = readShort();
  acpObject* pPrimitive = m_pRobot->getSubObject(primitiveIndex);
  aAssert(pPrimitive);

  char name[100];
  readString(name);

  addLogLine(name);

  short numProps = readShort();

  acpObject* pBehavior = 
  	m_pRobot->createBehavior(primitiveIndex, name);

  for (int i = 0; i < numProps; i++) {
    short propIndex = readShort();
    aPROPERTY_FLAGS flags = pPrimitive->getPropertyFlags(propIndex);

    // we use a different mechanism for callbacks
    if (flags & aPROPERTY_FLAG_CALLBACK) {

      int cbIndex = readInt();

      if (propIndex == m_nExecuteCBIndex) {
        acpRobotAgentExecute* executeCB = 
      	  new acpRobotAgentExecute(this, pBehavior, cbIndex);
        acpValue execute(executeCB);
      	pBehavior->setNamedValue("execute_callback", &execute);
      } else if (propIndex == m_nCompletionCBIndex) {
        acpRobotAgentCompletion* completionCB = 
      	  new acpRobotAgentCompletion(this, pBehavior, cbIndex);
        acpValue completion(completionCB);
        pBehavior->setNamedValue("completion_callback", &completion);
      }

    } else {
      pBehavior->readValue(propIndex, m_socket);
    }
  }

  short id = (short)(pBehavior->getNamedValue("unique_id")->getIntVal());

  // now it is built up so queue it
  m_pRobot->queueBehavior(pBehavior);

  writeShort(id);
  flush();

} // doBehavior method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doCallbacks()
{
  // start with the number of available callbacks (may be zero)
  int numCallbacks = m_callbacks.length();
  writeShort((short)numCallbacks);

  int i;
  for (i = 0; i < numCallbacks; i++) {
  
    // remove the callback from the list
    acpAgentCBHook* pCBHook = m_callbacks.removeHead();
    
    // write the callback id
    writeInt(pCBHook->getCBID());
    
    // write out any updated data
    pCBHook->writeUpdateData(m_socket);
    
    // clean up the callback
    delete pCBHook;
  }

  // ship it
  flush();

} // doCallbacks method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doWrite()
{
  // get index
  int address = readInt();
  short index = readShort();
  int type = readInt();

  // cast the address to an abstract object
  acpObject* pObject = (acpObject*)address;

  switch (type) {

  case aPROPERTY_FLAG_BOOL:
    {
      bool flag = (bool)readChar();
      acpValue writeVal(flag);
      pObject->setValue(index, &writeVal);
    }
    break;

  case aPROPERTY_FLAG_INT:
    {
      int i = readInt();
      //      printf("i %d\n", i);
      //      fflush(stdout);
      acpValue writeVal(i);
      pObject->setValue(index, &writeVal);
    }
    break;

  case aPROPERTY_FLAG_FLOAT:
    {
      float f = readFloat();
#ifdef aDEBUG
      char msg[100];
      sprintf(msg, "setting float value %f", f);
      addLogLine(msg);
#endif
      acpValue writeVal(f);
      pObject->setValue(index, &writeVal);
    }
    break;

  case aPROPERTY_FLAG_STRING:
    {
      char line[1000]; // ???
      readString(line);
      //      printf("s %s\n", line);
      //      fflush(stdout);
      acpValue writeVal(line);
      pObject->setValue(index, &writeVal);
    }
    break;

  } // switch

} // doWrite method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doRead()
{
  // get address, index and type
  int address = readInt();
  short index = readShort();
  int type = readInt();

  // cast the addres to an abstract object
  acpObject* pObject = (acpObject*)address;
  
  switch (type) {

    case aPROPERTY_FLAG_BOOL:
    {
      bool v = pObject->getValue(index)->getBoolVal();
      writeChar(v);
      break;
    }
  
    case aPROPERTY_FLAG_INT:
    {
      int i = pObject->getValue(index)->getIntVal();
      writeInt(i);
      break;
    }

    case aPROPERTY_FLAG_FLOAT:
    {
      float f = pObject->getValue(index)->getFloatVal();
#ifdef aDEBUG
      char msg[100];
      sprintf(msg, "read float value %f", f);
      addLogLine(msg);
#endif // aDEBUG
      writeFloat(f);
      break;
    }

    case aPROPERTY_FLAG_STRING:
    {
      const char* ps = pObject->getValue(index)->getStringVal();
      writeString(ps);
      break;
    }

  } // switch

  flush();

} // doRead method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::doTemp()
{
  // get index
  short index = readShort();
  float val = readFloat();
  
  acpValue* pVal = NULL;

  pVal = m_pRobot->getNamedValue("obj-camera-boom");

  if (pVal) {
    acpObject* pCamera = (acpObject*)pVal->getVoidPtrVal();
    acpValue newVal(val);
    pCamera->setValue(index, &newVal);
  }

} // doTemp method


/////////////////////////////////////////////////////////////////////
// doLinkUp

void acpRobotAgent::doLinkUp()
{
  // get next link up timeout interval
  m_linkUpInterval = readInt();

  // send the L reply to let the client know we are still here
  writeChar('L');
  flush();

  // store when the timeout was adjusted
  aIO_GetMSTicks(m_ioRef, &m_linkUpTime, NULL);

} // doLinkUp method


/////////////////////////////////////////////////////////////////////
// doDescription

void acpRobotAgent::doDescription()
{
  acpObject* pObject = readObject();
  short index = readShort();

  // cast the address to an abstract object
  acpProperty* pProperty = pObject->getProperty(index);

  acpString description(pProperty->getDescription());
#ifdef aDEBUG
  addLogLine(description);
#endif // aDEBUG

  writeString((char*)description);
  flush();

} // doDescrption method



/////////////////////////////////////////////////////////////////////

acpObject* acpRobotAgent::readObject()
{
  int o = readInt();

  printf("read object %X\n", o);

  aAssert(o != -1);

  acpObject* pObject = (acpObject*)o;

  return pObject;

} // readObject method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::writeObject (
  const acpObject* pObject
)
{
  writeInt((int)(long)pObject);

} // writeObject method



/////////////////////////////////////////////////////////////////////

short acpRobotAgent::readShort()
{
  char buf[sizeof(short)];
  short val = -1;
  if (m_socket 
      && !aStream_Read(m_ioRef, m_socket, buf, sizeof(short), NULL))
    val = aUtil_RetrieveShort(buf);

  return val;

} // readShort method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::writeShort (
  const short nVal
)
{
  if (m_buffer) {
    char buf[sizeof(short)];
    aUtil_StoreShort(buf, nVal);
    aStream_Write(m_ioRef, m_buffer, buf, sizeof(short), NULL);
  }

} // writeShort method



/////////////////////////////////////////////////////////////////////

int acpRobotAgent::readInt()
{
  char buf[sizeof(int)];
  int val = -1;
  if (m_socket 
      && !aStream_Read(m_ioRef, m_socket, buf, sizeof(int), NULL))
    val = aUtil_RetrieveInt(buf);

  return val;

} // readInt method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::writeInt(const int nVal)
{
  char buf[sizeof(int)];
  aUtil_StoreInt(buf, nVal);

  if (m_buffer)
    aStream_Write(m_ioRef, m_buffer, buf, sizeof(int), NULL);

} // writeInt method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::writeFloat(
  const float fVal)
{
  char buf[sizeof(float)];
  aUtil_StoreFloat(buf, fVal);

  if (m_buffer)
    aStream_Write(m_ioRef, m_buffer, buf, sizeof(float), NULL);

} // writeFloat method



/////////////////////////////////////////////////////////////////////

unsigned char acpRobotAgent::readChar()
{
  unsigned char val = 0xFF;
  if (m_socket) 
    aStream_Read(m_ioRef, m_socket, (char*)&val, 1, NULL);
  
  return val;

} // readChar method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::writeChar (
  unsigned char pChar
)
{
  if (m_buffer)
    aStream_Write(m_ioRef, m_buffer, (char*)&pChar, 1, NULL);

} // writeChar



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::readString (
  char* pString
)
{
  char* p = pString;
  aErr err;

  do {
    aStream_Read(m_ioRef, m_socket, p, 1, &err);
  } while ((err == aErrNone) && *p++);

} // readString method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::writeString (
  const char* pString
)
{
  if (m_buffer) {
    unsigned int len = aStringLen(pString) + 1;
    aStream_Write(m_ioRef, m_buffer, pString, len, NULL);
  }

} // writeString method



/////////////////////////////////////////////////////////////////////

float acpRobotAgent::readFloat()
{
  char buf[sizeof(float)];
  float f = -1.0f;

  if (m_socket
      && !aStream_Read(m_ioRef, m_socket, buf, sizeof(float), NULL))
    f = aUtil_RetrieveFloat(buf);

  return f;

} // readFloat method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::flush()
{
  if (m_buffer && m_socket)
    aStreamBuffer_Flush(m_ioRef, m_buffer, m_socket, NULL);

} // flush method



/////////////////////////////////////////////////////////////////////

void acpRobotAgent::welcome()
{
  // show a welcome message
  addLogLine("Robot Agent 1.0");

  char line[50];
  char num[24];

  aStringCopy(line, "addr: ");
  aUtil_FormatInetAddr(num, m_nInetAddr, NULL);
  aStringCat(line, num);
  addLogLine(line);

  aStringCopy(line, "port: ");
  aStringFromInt(num, m_nPort);
  aStringCat(line, num);
  addLogLine(line);

  aStringCopy(line, "for: ");
  aStringCat(line, m_pRobot->getNamedValue("name")->getStringVal());
  addLogLine(line);

  addLogLine("");

} // acpRobotAgent welcome method



/////////////////////////////////////////////////////////////////////
// This routine gets called once for each timer interval to
// allow us to update the robot, feedback, and display states.

void acpRobotAgent::TimeSlice()
{
  // handle any final setup now that we are up and running
  if (!m_bFinalized) {
    resetSocket();
    m_bFinalized = true;
  }

  // handle the robot side of the communications
  if (m_pRobot->getNamedValue("active")->getBoolVal()) {
    if (m_eMode != kActive) {
      setStatusText("Active");
      m_eMode = kActive;
    }
    // check for callbacks
    m_pRobot->handleCallbacks(1);
  } else {
    if (m_eMode != kInactive) {
      setStatusText("Not Active");
      m_eMode = kInactive;
    }
  }

  // check for client communications
  char c;
  aErr err;
  aStream_Read(m_ioRef, m_socket, &c, 1, &err);

  // EOF means nothing new happening
  if (err == aErrNone) {
    handleRequest(c);
  } else if (err == aErrIO) {
    resetSocket();
  }

  // handle client heartbeats where requested
  if (m_linkUpInterval) {
    unsigned long now;
    aIO_GetMSTicks(m_ioRef, &now, NULL);
    if (now > (m_linkUpTime + m_linkUpInterval)) {
      printf("link timeout expired\n");
      resetSocket();
    }
  }

} // acpRobotAgent TimeSlice method




/////////////////////////////////////////////////////////////////////
// acpRobotAgent::dialogIdle static method 
//
// Called periodically when the dialog for picking the Agent file
// is up.

aErr acpRobotAgent::dialogIdle(
  const void* ref
)
{
  acpRobotAgent* pAgent = (acpRobotAgent*)ref;

  aAssert(pAgent);

  pAgent->TimeSlice();
  
  return aErrNone;
  
} // acpRobotAgent::dialogIdle static method



/////////////////////////////////////////////////////////////////////
// win32_acpRobotAgent dumpFilter method
//
// used to display only the .xml files in the pick file dialog

aBool acpRobotAgent::dumpFilter(
  const char* pFilename,
  const unsigned long nSize
)  
{
  char extension[aFILE_NAMEMAXCHARS];
  
  aUtil_GetFileExtension(extension, pFilename, NULL);
  if (!aStringCompare(extension, ".xml"))
    return aTrue;
  
  return aFalse;

} // acpRobotAgent dumpFilter method



/////////////////////////////////////////////////////////////////////
// This routine launches a browser window pointing to the Robot
// API's mini web server.

void acpRobotAgent::cmdAPIView()
{
  aErr err = aErrNone;
  unsigned long address;

  if (err == aErrNone)
    aIO_GetInetAddr(m_ioRef, &address, &err);

  if (err == aErrNone) {
    char url[100];
    char ip[20];
    aStringCopy(url, "http://");
    if (!aUtil_FormatInetAddr(ip, address, &err)) {
      aStringCat(url, ip);
      aStringCat(url, ":8000/apiview");    
      aBrowser_LaunchURL(m_uiRef, url, &err);
    }
  }

  aAssert(err == aErrNone);

} // acpRobotAgent cmdAPIView method



/////////////////////////////////////////////////////////////////////
// This routine dispatches the command the proper handling routine.

void acpRobotAgent::dispatchCommand(const int nCmd)
{
  switch (nCmd) {
 
  case aCMD_DOAPIVIEW:
    cmdAPIView();
    break;

  case aCMD_DOEXIT:
    cmdExit();
    break;

  } // switch

} // acpRobotAgent dispatchCommand



/////////////////////////////////////////////////////////////////////
// acpRobotAgentHB heartbeat callback 
//
// Called each time the heartbeat status changes.  0 means HB off,
// 1 means HB on.

aErr acpRobotAgentHB::call()
{
  bool bHB = m_pcAgent->m_pRobot->getNamedValue(
  		"heartbeat_status")->getBoolVal();

  if (bHB != m_pcAgent->m_bHB) {
    m_pcAgent->m_bHB = bHB;
    m_pcAgent->updateHeartbeat();
  }

  return aErrNone;

} // acpRobotAgentHB call method



/////////////////////////////////////////////////////////////////////
// acpRobotAgentExecute execute callback 
//

aErr acpRobotAgentExecute::call()
{
  m_pcAgent->addCallbackHook(this);

  // this gets thrown away after this routine is called
  // so we clear it out here to avoid using it on accident
  // later
  m_pBehavior = NULL;

  return aErrNone;

} // acpRobotAgentExecute call method



/////////////////////////////////////////////////////////////////////
// acpRobotAgentExecute completion callback 
//

aErr acpRobotAgentExecute::writeUpdateData(
  aStreamRef dest)
{
  // execute doesn't modify the behavior at all
  m_pcAgent->writeShort(0);

  return aErrNone;

} // acpRobotAgentExecute call method



/////////////////////////////////////////////////////////////////////
// acpRobotAgentCompletion completion callback 
//

aErr acpRobotAgentCompletion::call()
{
  // grab the status and hold on to it.
  m_nStatus =
    m_pBehavior->getNamedValue("completion_status")->getIntVal();

  m_pcAgent->addCallbackHook(this);

  // this gets thrown away after this routine is called
  // so we clear it out here to avoid using it on accident
  // later
  m_pBehavior = NULL;

  return aErrNone;

} // acpRobotAgentCompletion call method



/////////////////////////////////////////////////////////////////////
// acpRobotAgentCompletion completion callback 
//

aErr acpRobotAgentCompletion::writeUpdateData(
  aStreamRef dest)
{
  // write the number of modified behavior values
  m_pcAgent->writeShort(1);

  // completion status value
  m_pcAgent->writeShort((short)m_pcAgent->m_nCompletionStatusIndex);
  m_pcAgent->writeInt(m_nStatus);

  return aErrNone;

} // acpRobotAgentCompletion call method
