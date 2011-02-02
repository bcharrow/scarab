/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotClient.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the acpRobotClient object.           //
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
#include "acpShort.h"
#include "acpInt32.h"
#include "acpFloat.h"
#include "acpException.h"
#include "acpProperty.h"
#include "acpRobotClient.h"


 
/////////////////////////////////////////////////////////////////////

acpRobotClient::acpRobotClient() :
  m_bConnected(false),
  m_nTimeout(5000),
  m_ipaddress(0),
  m_ipport(8008),
  m_stream(NULL),
  m_bufferStream(NULL)
{
  aErr e;
  if (aIO_GetLibRef(&m_ioRef, &e))
    throw acpException(e, "getting aIO library"); 

  if (aStreamBuffer_Create(m_ioRef, 256, &m_bufferStream, &e))
    throw acpException(e, "creating buffer stream"); 
}


/////////////////////////////////////////////////////////////////////

acpRobotClient::~acpRobotClient() 
{
  aErr e;
  
  if (m_bConnected)
    bye();

  if (m_stream && aStream_Destroy(m_ioRef, m_stream, &e))
    throw acpException(e, "closing TCP/IP stream");
  if (m_bufferStream && aStream_Destroy(m_ioRef, m_bufferStream, &e))
    throw acpException(e, "closing buffer stream");
  if (aIO_ReleaseLibRef(m_ioRef, &e))
    throw acpException(e, "releasing aIO library");
}


/////////////////////////////////////////////////////////////////////

int acpRobotClient::connect(
  const int nAddress,
  const int nPort,
  const int nTimeout) 
{
  // save the settings
  m_ipaddress = nAddress;
  m_ipport = (unsigned short)nPort;
  m_nTimeout = nTimeout;

#ifdef aDEBUG
  char ipAddressString[16];
  aUtil_FormatInetAddr(ipAddressString, m_ipaddress, NULL);
  printf("Attempting to connect to robot at %s:%d with timeout %d.\n",
  	 ipAddressString,
  	 m_ipport, (int)m_nTimeout);
#endif // aDEBUG

  aErr e;
  aStream_CreateSocket(m_ioRef, 
  		       m_ipaddress,
  		       m_ipport,
  		       aFalse,
  		       &m_stream,
  		       &e);
  int rootID = 0;

  if ((e == aErrNone) && (rootID = ack())) {
    m_bConnected = true;
    printf("Connected successfully!\n");
  } else {
    aStream_Destroy(m_ioRef, m_stream, &e);
  }

  return rootID;
}


/////////////////////////////////////////////////////////////////////
// reset
//
// Close down the socket connection and handle any state information
// associated with an open socket.

void acpRobotClient::reset()
{
  m_bConnected = false;

  if (m_stream) {
    aErr e;
    if (aStream_Destroy(m_ioRef, m_stream, &e))
      throw acpException(e, "unable to reset client socket");
    m_stream = NULL;
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::writeBoolProperty(
  const int nObjectID,
  const int nPropIndex,
  const bool value)
{
  sendChar('W');
  sendInt(nObjectID);
  sendShort((short)nPropIndex);
  sendInt(aPROPERTY_FLAG_BOOL);
  sendChar((char)value, true);
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::writeIntProperty(
  const int nObjectID,
  const int nPropIndex,
  const int value)
{
  sendChar('W');
  sendInt(nObjectID);
  sendShort((short)nPropIndex);
  sendInt(aPROPERTY_FLAG_INT);
  sendInt(value, true);
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::writeFloatProperty(
  const int nObjectID,
  const int nPropIndex,
  const float value)
{
  sendChar('W');                         //  1
  sendInt(nObjectID);                    //  5
  sendShort((short)nPropIndex);          //  7
  sendInt(aPROPERTY_FLAG_FLOAT);         // 11
  sendFloat(value, true);                // 15
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::writeStringProperty(
  const int nObjectID,
  const int nPropIndex,
  const char* pValue)
{
  sendChar('W');                         //  1
  sendInt(nObjectID);                    //  5
  sendShort((short)nPropIndex);          //  7
  sendInt(aPROPERTY_FLAG_STRING);        // 11
  sendString(pValue, true);              // variable
}


/////////////////////////////////////////////////////////////////////

float acpRobotClient::readFloatProperty(
  const int nObjectID,
  const int nPropIndex)
{
  sendChar('R');                         //  1
  sendInt(nObjectID);                    //  5
  sendShort((short)nPropIndex);          //  7
  sendInt(aPROPERTY_FLAG_FLOAT, true);   // 11
  return nextFloat();
}


/////////////////////////////////////////////////////////////////////

int acpRobotClient::readIntProperty(
  const int nObjectID,
  const int nPropIndex)
{
  sendChar('R');                         //  1
  sendInt(nObjectID);                    //  5
  sendShort((short)nPropIndex);          //  7
  sendInt(aPROPERTY_FLAG_INT, true);     // 11
  return nextInt();
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::enumProperties(
  const int objectID,
  robotPropEnumProc enumerator,
  void* vpRef)
{
  if (!m_bConnected)
    return;

  // make the request for the properties
  sendChar('P');
#ifdef aDEBUG
  printf("enumerating object %X\n", objectID);
#endif // aDEBUG
  sendInt(objectID, true);

  // get the reply
  int nProperties = nextShort();
#ifdef aDEBUG
  printf("enumerating %d properties\n", nProperties);
#endif // aDEBUG
  for (int i = 0; i < nProperties; i++) {
    acpString name;
    nextString(&name);
    int typeFlags = nextInt();
    if (enumerator)
      enumerator((char*)name, i, typeFlags, vpRef);
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::getObjectName(
  acpString* pName,
  const int nObjectID) 
{
  sendChar('N');                         //  1
  sendInt(nObjectID, true);              //  5

  // grab the description
  nextString(pName);
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::getPropertyDescription(
  acpString* pDescription,
  const int nObjectID,
  const int nPropIndex) 
{
  sendChar('D');                         //  1
  sendInt(nObjectID);                    //  5
  sendShort((short)nPropIndex, true);    //  7

  // grab the description
  nextString(pDescription);
}


/////////////////////////////////////////////////////////////////////

bool acpRobotClient::sendChar(
  const char data,
  const bool bFinal
) 
{
  aErr e;
  aStream_Write(m_ioRef, m_bufferStream, &data, 1, &e);
  if (e != aErrNone) {
    printf("buffer stream write call failed %d\n", e);
  }
  if (bFinal)
    aStreamBuffer_Flush(m_ioRef,
			m_bufferStream,
			m_stream,
			&e);
  aAssert(e == aErrNone);
  return (e == aErrNone);
}


/////////////////////////////////////////////////////////////////////

bool acpRobotClient::sendShort(
  const short data,
  const bool bFinal
) 
{
  if (m_bConnected) {
    acpShort shortVal(data);
    shortVal.writeToStream(m_bufferStream);
    aErr e = aErrNone;
    if (bFinal)
      aStreamBuffer_Flush(m_ioRef,
			  m_bufferStream,
			  m_stream,
			  &e);
    aAssert(e == aErrNone);
    if (e == aErrNone)
      return true;
  }
  return false;
}


/////////////////////////////////////////////////////////////////////

bool acpRobotClient::sendInt(
  const int data,
  const bool bFinal
) 
{
  if (m_bConnected) {
    acpInt32 intVal(data);
    intVal.writeToStream(m_bufferStream);
    aErr e = aErrNone;
    if (bFinal)
      aStreamBuffer_Flush(m_ioRef,
			  m_bufferStream,
			  m_stream,
			  &e);
    aAssert(e == aErrNone);
    if (e == aErrNone)
      return true;
    else {
      printf("buffer flush error: %d\n", e);
      m_bConnected = false;
    }
  }
  return false;
}


/////////////////////////////////////////////////////////////////////

bool acpRobotClient::sendFloat(
  const float data,
  const bool bFinal
)
{
  if (m_bConnected) {
    acpFloat floatVal(data);
    floatVal.writeToStream(m_bufferStream);
    aErr e = aErrNone;
    if (bFinal)
      aStreamBuffer_Flush(m_ioRef,
			  m_bufferStream,
			  m_stream,
			  &e);

    aAssert(e == aErrNone);
    if (e == aErrNone)
      return true;
  }
  return false;
}


/////////////////////////////////////////////////////////////////////

bool acpRobotClient::sendString(
  const char* data,
  const bool bFinal
) 
{
  if (m_bConnected) {
    int len = aStringLen(data);
    aErr e;
    aStream_Write(m_ioRef, m_bufferStream, data, len + 1, &e);
    aAssert(e == aErrNone);
    if (bFinal)
      aStreamBuffer_Flush(m_ioRef,
			  m_bufferStream,
			  m_stream,
			  &e);
    if (e == aErrNone)
      return true;
  }

  return false;
}


/////////////////////////////////////////////////////////////////////

int acpRobotClient::nextByte() {

  aErr e;
  unsigned long now, done;
  unsigned char c;

  // initialize
  c = 0;

  if (aIO_GetMSTicks(m_ioRef, &now, &e))
    throw acpException(e, "getting time");

  done = now + m_nTimeout;

  while ((e == aErrNone) && (now < done)) {
    aStream_Read(m_ioRef, m_stream, (char*)&c, 1, &e);
    if (e == aErrNone) {
      return c;
    } else if (e == aErrNotReady) {
      aIO_GetMSTicks(m_ioRef, &now, &e);
    } else {
      throw acpException(e, "stream read error");
    }
  }

  return -1;
}


/////////////////////////////////////////////////////////////////////

short acpRobotClient::nextShort() 
{
  unsigned char buf[sizeof(short)];
  if (m_bConnected) {
    buf[0] = (unsigned char)nextByte();
    buf[1] = (unsigned char)nextByte();
    return(aUtil_RetrieveShort((char*)buf));
  }

  return -1;
}


/////////////////////////////////////////////////////////////////////

int acpRobotClient::nextInt() 
{
  unsigned char buf[sizeof(int)];
  if (m_bConnected) {
    buf[0] = (unsigned char)nextByte();
    buf[1] = (unsigned char)nextByte();
    buf[2] = (unsigned char)nextByte();
    buf[3] = (unsigned char)nextByte();
    return(aUtil_RetrieveInt((char*)buf));
  }

  return -1;
}


/////////////////////////////////////////////////////////////////////

float acpRobotClient::nextFloat() 
{
  unsigned char buf[sizeof(float)];
  if (m_bConnected) {
    for (int i = 0; i < (int)sizeof(float); i++) {
      int v = nextByte();
      if (v == -1)
	return -1;
      buf[i] = (unsigned char)v;
    }
    return(aUtil_RetrieveFloat((char*)buf));
  }

  return -1;
}


/////////////////////////////////////////////////////////////////////

bool acpRobotClient::nextString(
  acpString* pString
) 
{
  *pString = ""; // initialize

  int b = -1;
  if (m_bConnected) {
    do {
      b = nextByte();
      if (b > 0)
        *pString += (char)b;
    } while ((b != 0) && (b != -1));
  }
  if (b != -1)
    return true;
  
  return false;
}


/////////////////////////////////////////////////////////////////////

int acpRobotClient::ack() 
{
  int id = 0;

  if (sendChar('A', true)) {

    int reply;

    // get the next byte we get back
    do {
      reply = nextByte();
    } while (reply == -1);

    if (reply == 'A') {
      m_bConnected = true;
      id = nextInt();
#ifdef aDEBUG
      printf("root object ID = %X\n", id);
#endif // aDEBUG

#if 1
      // flush out any garbage in the pipe that we may have 
      // requested before (like multiple ACKs)
      do {
	reply = nextByte();
	if (reply != -1)
	  printf("flushing %d\n", reply);
      } while (reply != -1);
#endif

    }
    else if (reply != 'B')
      throw acpException(aErrBusy, "aRobot server is busy");
  }

  if (id == -1)
    m_bConnected = false;

  return id;
}


/////////////////////////////////////////////////////////////////////

void acpRobotClient::bye() 
{
  aErr e;
  static const char c = 'X';
  aStream_Write(m_ioRef, m_stream, &c, 1, &e);

  // give things time to close before we continue and possibly
  // tear down the stream
  if (e == aErrNone)
    aIO_MSSleep(m_ioRef, 100, &e);
}


/////////////////////////////////////////////////////////////////////

bool acpRobotClient::linkUp(
  const int nMSTimeout) 
{
  if (m_bConnected) {

    // send the message and the new timeout value
    sendChar('L');
    sendInt(nMSTimeout, true);

    // await a reply to make sure there is someone out there
    int reply = nextByte();
    if (reply != 'L') {
#ifdef aDEBUG
      printf("linkUp expected \'L\', but got %c, %d\n", reply, reply);
#endif // aDEBUG
      m_bConnected = false;
    }
  }
    
  return m_bConnected;
}
