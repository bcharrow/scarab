/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaAgent.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GarciaAgent application      //
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
#include "acpGarciaAgent.h"
#include "aUtil.h"



/////////////////////////////////////////////////////////////////////

acpGarciaAgent::acpGarciaAgent() :
  m_bFinalized(false),
  m_bHB(false),
  m_nTimerSetting(10),
  m_batteryCheck(0),
  m_lastStatus(0),
  m_logView(NULL),
  m_nPort(8008),
  m_socket(NULL),
  m_buffer(NULL),
  m_eMode(kInit),
  m_bConnected(false),
  m_nExecuteCBIndex(-1),
  m_nCompletionCBIndex(-1),
  m_nCompletionStatusIndex(-1)
{
  // install a heartbeat callback object
  acpValue hbVal(new acpGarciaAgentHB(this));
  m_garcia.setNamedValue("heartbeat-callback", &hbVal);

  if (aIO_GetLibRef(&m_ioRef, NULL))
     die("unable to open aIO library");
  
  if (aUI_GetLibRef(&m_uiRef, NULL))
     die("unable to open aUI library");
  
  aErr err;
  aIO_GetInetAddr(m_ioRef, &m_nInetAddr, &err);
  aAssert(err == aErrNone);

  if (aStreamBuffer_Create(m_ioRef, 1024, &m_buffer, NULL))
    die("unable to create output buffer");

} // acpGarciaAgent constructor



/////////////////////////////////////////////////////////////////////

acpGarciaAgent::~acpGarciaAgent()
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
  if (m_ioRef) {
    aIO_ReleaseLibRef(m_ioRef, NULL);
    m_ioRef = NULL;
  }
  if (m_uiRef) {
    aUI_ReleaseLibRef(m_uiRef, NULL);
    m_uiRef = NULL;
  }

} // acpGarciaAgent destructor



/////////////////////////////////////////////////////////////////////
// behavior completion callback

aErr acpGarciaAgent::taskComplete (
  acpGarcia* pGarcia,
  acpObject* pBehavior
)
{
  char line[100];
  char num[10];

  acpGarciaAgent* pAgent = 
    (acpGarciaAgent*)pGarcia->getNamedValue("app-object")->getVoidPtrVal();

  if (!pAgent)
    return aErrUnknown;

  pAgent->m_lastStatus = 
    pBehavior->getNamedValue("completion-status")->getIntVal();

  // show some status
  aStringCopy(line, "Agent completed with status ");
  aStringFromInt(num, pAgent->m_lastStatus);
  pAgent->addLogLine(line);

  return aErrNone;

} // acpGarciaAgent taskComplete method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::resetSocket()
{
  // clean out an old socket if present
  if (m_socket) {
    aStream_Destroy(m_ioRef, m_socket, NULL);
    m_socket = NULL;
    m_bConnected = false;
  }

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

} // acpGarciaAgent resetSocket method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::handleRequest (
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

  case 'O':
    doSubObjects();
    break;

  case 'P':
    doProperties();
    break;

  case 'C':
    doCallbacks();
    break;

  case 'W':
    doWrite();
    break;

  case 'R':
    doRead();
    break;

  case 'T':
    doTemp();
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

void acpGarciaAgent::respond (
  const char* pResponse
)
{
  if (m_socket) {
    unsigned int len = aStringLen(pResponse); 
    aStream_Write(m_ioRef, m_socket, pResponse, len, NULL);
  }

} // respond method



/////////////////////////////////////////////////////////////////////

aErr acpGarciaAgent::propertyEnum (
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS typeFlags,
  void* vpRef
)
{
  acpGarciaAgent* pAgent = (acpGarciaAgent*)vpRef;

  if (!aStringCompare(pName, "execute-callback"))
    pAgent->m_nExecuteCBIndex = nIndex;
  else if (!aStringCompare(pName, "completion-callback"))
    pAgent->m_nCompletionCBIndex = nIndex;
  else if (!aStringCompare(pName, "completion-status"))
    pAgent->m_nCompletionStatusIndex = nIndex;

  pAgent->writeString(pName);
  pAgent->writeInt(typeFlags);

  return aErrNone;

} // propertyEnum method



/////////////////////////////////////////////////////////////////////

aErr acpGarciaAgent::subObjectEnum (
  acpObject& object,
  void* vpRef
)
{
  acpGarciaAgent* pAgent = (acpGarciaAgent*)vpRef;

  // for each sub-object, write it's class and name
  // for each sub-object, write its address

  pAgent->writeString(object.getNamedValue("classname")->getStringVal());
  pAgent->writeString(object.getNamedValue("name")->getStringVal());
  pAgent->writeInt((int)(long)&object);

  return aErrNone;

} // subObjectEnum method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::doAck()
{
  addLogLine("connection started");

  m_bConnected = true;

// FIX -- what is busy???

  if (1) {
    writeChar('A');
    writeInt((int)(long)&m_garcia);
  } else {
    writeChar('B');
  }

  flush();

} // doAck method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::doSync()
{
  // properties
  writeShort((short)m_garcia.numProperties());
  m_garcia.enumProperties(propertyEnum, this);

  // sub-objects
  writeShort((short)m_garcia.numSubObjects());
  m_garcia.enumSubObjects(subObjectEnum, this);

  // flush does the entire sync in a single (or minimal number of)
  // TCP packets
  flush();

} // doSync method


/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::doSubObjects()
{
  acpObject* pObject = readObject();
  
  writeShort((aShort)pObject->numSubObjects());
  pObject->enumSubObjects(subObjectEnum, this);

  flush();

} // doSubObjects method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::doProperties()
{
  acpObject* pObject = readObject();

  writeShort((aShort)pObject->numProperties());
  pObject->enumProperties(propertyEnum, this);

  flush();

} // doProperties method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::doBehavior()
{
  // get the primitive to perform
  short primitiveIndex = readShort();
  acpObject* pPrimitive = m_garcia.getSubObject(primitiveIndex);
  aAssert(pPrimitive);

  char name[100];
  readString(name);

  addLogLine(name);

  short numProps = readShort();

  acpObject* pBehavior = 
  	m_garcia.createBehavior(primitiveIndex, name);

  for (int i = 0; i < numProps; i++) {
    short propIndex = readShort();
    aPROPERTY_FLAGS flags = pPrimitive->getPropertyFlags(propIndex);

    // we use a different mechanism for callbacks
    if (flags & aPROPERTY_FLAG_CALLBACK) {

      int cbIndex = readInt();

      if (propIndex == m_nExecuteCBIndex) {
        acpGarciaAgentExecute* executeCB = 
      	  new acpGarciaAgentExecute(this, pBehavior, cbIndex);
        acpValue execute(executeCB);
      	pBehavior->setNamedValue("execute-callback", &execute);
      } else if (propIndex == m_nCompletionCBIndex) {
        acpGarciaAgentCompletion* completionCB = 
      	  new acpGarciaAgentCompletion(this, pBehavior, cbIndex);
        acpValue completion(completionCB);
        pBehavior->setNamedValue("completion-callback", &completion);
      }

    } else {
      pBehavior->readValue(propIndex, m_socket);
    }
  }

  short id = (short)(pBehavior->getNamedValue("unique-id")->getIntVal());

  // now it is built up so queue it
  m_garcia.queueBehavior(pBehavior);

  writeShort(id);
  flush();

} // doBehavior method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::doCallbacks()
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

void acpGarciaAgent::doWrite()
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
      addLogLine("setting bool value");
      bool flag = (bool)readChar();
      acpValue writeVal(flag);
      pObject->setValue(index, &writeVal);
    }
    break;

  case aPROPERTY_FLAG_INT:
    {
      addLogLine("setting int value");
      int i = readInt();
      acpValue writeVal(i);
      pObject->setValue(index, &writeVal);
    }
    break;

  case aPROPERTY_FLAG_FLOAT:
    {
      addLogLine("setting float value");
      float f = readFloat();
      acpValue writeVal(f);
      pObject->setValue(index, &writeVal);
    }
    break;

  case aPROPERTY_FLAG_STRING:
    {
      char line[1000]; // FIX???
      readString(line);
      acpValue writeVal(line);
      pObject->setValue(index, &writeVal);
    }
    break;

  } // switch

} // doWrite method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::doRead()
{
  // get address, index and type
  int address = readInt();
  short index = readShort();
  int type = readInt();

  // cast the address to an abstract object
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
      int i;
      aMemCopy(&i, &f, sizeof(int));
      writeInt(i); // hack that breaks on 64bit FIXME
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

void acpGarciaAgent::doTemp()
{
  // get index
  short index = readShort();
  float val = readFloat();
  
  acpValue* pVal = NULL;

  pVal = m_garcia.getNamedValue("obj-camera-boom");

  if (pVal) {
    acpObject* pCamera = (acpObject*)pVal->getVoidPtrVal();
    acpValue newVal(val);
    pCamera->setValue(index, &newVal);
  }

} // doTemp method



/////////////////////////////////////////////////////////////////////

acpObject* acpGarciaAgent::readObject()
{
  acpObject* pObject = (acpObject*)readInt();

  aAssert(pObject);

  return pObject;

} // readObject method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::writeObject (
  const acpObject* pObject
)
{
  writeInt((int)(long)pObject);

} // writeObject method



/////////////////////////////////////////////////////////////////////

short acpGarciaAgent::readShort()
{
  char buf[sizeof(short)];
  short val = -1;
  if (m_socket 
      && !aStream_Read(m_ioRef, m_socket, buf, sizeof(short), NULL))
    val = aUtil_RetrieveShort(buf);

  return val;

} // readShort method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::writeShort (
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

int acpGarciaAgent::readInt()
{
  char buf[sizeof(int)];
  int val = -1;
  if (m_socket 
      && !aStream_Read(m_ioRef, m_socket, buf, sizeof(int), NULL))
    val = aUtil_RetrieveInt(buf);

  return val;

} // readInt method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::writeInt(const int nVal)
{
  char buf[sizeof(int)];
  aUtil_StoreInt(buf, nVal);

  if (m_buffer)
    aStream_Write(m_ioRef, m_buffer, buf, sizeof(int), NULL);

} // readInt method



/////////////////////////////////////////////////////////////////////

unsigned char acpGarciaAgent::readChar()
{
  unsigned char val = 0xFF;
  if (m_socket) 
    aStream_Read(m_ioRef, m_socket, (char*)&val, 1, NULL);
  
  return val;

} // readChar method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::writeChar (
  unsigned char pChar
)
{
  if (m_buffer)
    aStream_Write(m_ioRef, m_buffer, (char*)&pChar, 1, NULL);

} // writeChar



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::readString (
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

void acpGarciaAgent::writeString (
  const char* pString
)
{
  if (m_buffer) {
    unsigned int len = aStringLen(pString) + 1;
    aStream_Write(m_ioRef, m_buffer, pString, len, NULL);
  }

} // writeString method



/////////////////////////////////////////////////////////////////////

float acpGarciaAgent::readFloat()
{
  char buf[sizeof(float)];

  if (m_socket) {
    if (!aStream_Read(m_ioRef, m_socket, buf, sizeof(float), NULL)) {
      char* data = (char*)buf;

      union {
        char  c[4];
        float f;
      } tmp;
      register int i;
      for (i=4; i--; ) tmp.c[3 - i] = data[i];

      return tmp.f;
    }
  }
  
  return -1;

} // readString method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::flush()
{
  if (m_buffer && m_socket)
    aStreamBuffer_Flush(m_ioRef, m_buffer, m_socket, NULL);

} // flush method



/////////////////////////////////////////////////////////////////////

void acpGarciaAgent::welcome()
{
  // show a welcome message
  addLogLine("Garcia Agent 1.0");

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

  addLogLine("");

} // acpGarciaAgent welcome method



/////////////////////////////////////////////////////////////////////
// This routine gets called once for each timer interval to
// allow us to update the robot, feedback, and display states.

void acpGarciaAgent::TimeSlice()
{
  // handle any final setup now that we are up and running
  if (!m_bFinalized) {
    resetSocket();
    m_bFinalized = true;
  }

  // handle the garcia side of the communications
  if (m_garcia.getNamedValue("active")->getBoolVal()) {
    if (!m_eMode != kActive) {
      setStatusText("Active");
      m_eMode = kActive;
    }
    // check for callbacks
    m_garcia.handleCallbacks(1);
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

} // acpGarciaAgent TimeSlice method




/////////////////////////////////////////////////////////////////////
// acpGarciaAgent::dialogIdle static method 
//
// Called periodically when the dialog for picking the Agent file
// is up.

aErr acpGarciaAgent::dialogIdle(
  const void* ref
)
{
  acpGarciaAgent* pAgent = (acpGarciaAgent*)ref;

  aAssert(pAgent);

  pAgent->TimeSlice();
  
  return aErrNone;
  
} // acpGarciaAgent::dialogIdle static method



/////////////////////////////////////////////////////////////////////
// win32_acpGarciaAgent dumpFilter method
//
// used to display only the .xml files in the pick file dialog

aBool acpGarciaAgent::dumpFilter(
  const char* pFilename,
  const unsigned long nSize
)  
{
  char extension[aFILE_NAMEMAXCHARS];
  
  aUtil_GetFileExtension(extension, pFilename, NULL);
  if (!aStringCompare(extension, ".xml"))
    return aTrue;
  
  return aFalse;

} // acpGarciaAgent dumpFilter method



/////////////////////////////////////////////////////////////////////
// This routine launches a browser window pointing to the Garcia
// API's mini web server.

void acpGarciaAgent::cmdAPIView()
{
  aErr err = aErrNone;
  unsigned long address;

  if (err == aErrNone)
    aIO_GetInetAddr(m_ioRef, &address, &err);

  if (err == aErrNone) {
    char url[100];
    char ip[20];
    aStringCopy(url, "http://");
    aUtil_FormatInetAddr(ip, address, NULL);
    aStringCat(url, ip);
    aStringCat(url, ":8000/apiview");    
    aBrowser_LaunchURL(m_uiRef, url, &err);
  }

  aAssert(err == aErrNone);

} // acpGarciaAgent cmdAPIView method



/////////////////////////////////////////////////////////////////////
// This routine dispatches the command the proper handling routine.

void acpGarciaAgent::dispatchCommand(const int nCmd)
{
  switch (nCmd) {
 
  case aCMD_DOAPIVIEW:
    cmdAPIView();
    break;

  case aCMD_DOEXIT:
    cmdExit();
    break;

  } // switch

} // acpGarciaAgent dispatchCommand



/////////////////////////////////////////////////////////////////////
// acpGarciaAgentHB heartbeat callback 
//
// Called each time the heartbeat status changes.  0 means HB off,
// 1 means HB on.

aErr acpGarciaAgentHB::call()
{
  bool bHB = m_pcAgent->m_garcia.getNamedValue(
  		"heartbeat-status")->getBoolVal();

  if (bHB != m_pcAgent->m_bHB) {
    m_pcAgent->m_bHB = bHB;
    m_pcAgent->updateHeartbeat();
  }

  return aErrNone;

} // acpGarciaAgentHB call method



/////////////////////////////////////////////////////////////////////
// acpGarciaAgentExecute execute callback 
//

aErr acpGarciaAgentExecute::call()
{
  m_pcAgent->addCallbackHook(this);

  // this gets thrown away after this routine is called
  // so we clear it out here to avoid using it on accident
  // later
  m_pBehavior = NULL;

  return aErrNone;

} // acpGarciaAgentExecute call method



/////////////////////////////////////////////////////////////////////
// acpGarciaAgentExecute completion callback 
//

aErr acpGarciaAgentExecute::writeUpdateData(
  aStreamRef dest)
{
  // execute doesn't modify the behavior at all
  m_pcAgent->writeShort(0);

  return aErrNone;

} // acpGarciaAgentExecute call method



/////////////////////////////////////////////////////////////////////
// acpGarciaAgentCompletion completion callback 
//

aErr acpGarciaAgentCompletion::call()
{
  // grab the status and hold on to it.
  m_nStatus = m_pBehavior->getNamedValue("completion-status")->getIntVal();

  m_pcAgent->addCallbackHook(this);

  // this gets thrown away after this routine is called
  // so we clear it out here to avoid using it on accident
  // later
  m_pBehavior = NULL;

  return aErrNone;

} // acpGarciaAgentCompletion call method



/////////////////////////////////////////////////////////////////////
// acpGarciaAgentCompletion completion callback 
//

aErr acpGarciaAgentCompletion::writeUpdateData(
  aStreamRef dest)
{
  // write the number of modified behavior values
  m_pcAgent->writeShort(1);

  // completion status value
  m_pcAgent->writeShort((short)m_pcAgent->m_nCompletionStatusIndex);
  m_pcAgent->writeInt(m_nStatus);

  return aErrNone;

} // acpGarciaAgentCompletion call method
