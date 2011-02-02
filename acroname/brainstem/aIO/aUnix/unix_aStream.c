/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: unix_aStream.c                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: implementation of serial stream I/O routines for   */
/*		Unix.						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2008. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifdef aUNIX

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>      /* for serial stuff */
#include <fcntl.h>       /* for serial stuff */
#include <sys/signal.h>  /* for serial stuff */
#include <termios.h>     /* for serial stuff */
#include <sys/types.h>   /* for serial stuff */
#include <sys/socket.h>  /* for socket stuff */
#include <netdb.h>       /* for socket stuff */
#include <netinet/tcp.h> /* for socket stuff */
#include <netinet/in.h>  /* for socket stuff */
#include <arpa/inet.h>   /* for socket stuff */
#include <sys/ioctl.h>   /* for socket stuff */
#include <sys/time.h>    /* for socket stuff */

#include "unix_aIO.h"
#include "aIO.h"
#include "aStream.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

typedef struct aUnixSerialStream {

  unsigned int		nBaudRate;
  char			devName[aFILE_NAMEMAXCHARS];
  int                   nSerialDevice;
  struct termios        oldtio;
  struct termios        newtio;

  int			check;

} aUnixSerialStream;

#define aSERIALCHECK  0xBEEF

static aErr sSerialStreamGet(char* pData, void* ref);
static aErr sSerialStreamPut(char* pData, void* ref);
static aErr sSerialStreamDelete(void* ref);
static aErr sStreamOpenSerial(aUnixSerialStream* pStreamData);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define aSOCKETOPEN		0x0001
#define aSOCKETACCEPTED		0x0002
#define aSOCKETLISTENING	0x0004
#define aSOCKETNEEDSCONNECT	0x0008

typedef struct aUnixSocketStream {

  aUnixIOLib* 		pLib;

  aUNIXIOSocket*        pServer;

  struct sockaddr_in	clientAddr;
  int			client;

  int			flags;

  int			check;

} aUnixSocketStream;

#define aSOCKETCHECK  0xDEAD

static aErr sSocketStreamPrepare(aUnixSocketStream* pStreamData);
static aErr sSocketStreamGet(char* pData, void* ref);
static aErr sSocketStreamPut(char* pData, void* ref);
static aErr sSocketStreamWrite(const char* pData, 
			       const unsigned long nSize,
			       void* ref);
static aErr sSocketStreamDelete(void* ref);
static aErr sSocketListen(aUnixSocketStream* pStreamData);
static aErr sServerSocketAcquire(aUnixIOLib* pIOLib,
			         unsigned int address,
			         unsigned short port,
				 aUNIXIOSocket** ppServer);
static aErr sServerSocketRelease(aUNIXIOSocket* pServer);
static aErr sSetSocketOptions(const int iSocket);

#ifdef aDEBUG
#define SHOWERRNO(msg) printf("ERROR %s: %d\n", msg, errno)
#else /* aDEBUG */
#define SHOWERRNO(msg)
#endif /* aDEBUG */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSerialStreamGet
 */

aErr sSerialStreamGet(char* pData,
		      void* ref)
{
  aUnixSerialStream* pStreamData = (aUnixSerialStream*)ref;
  ssize_t count = 0;

  /* check for valid stream data */
  if ((pStreamData == NULL) ||
      (pStreamData->check != aSERIALCHECK))
    return aErrParam;

  count = read(pStreamData->nSerialDevice, pData, 1);

  if (count == 1)
    return aErrNone;

  if (count == 0)
    return aErrNotReady;

  return aErrIO;

} /* end of sSerialStreamGet */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSerialStreamPut
 */

aErr sSerialStreamPut(char* pData,
		      void* ref)
{
  aUnixSerialStream* pStreamData = (aUnixSerialStream*)ref;
  ssize_t written = 0;

  /* check for valid stream data */
  if ((pStreamData == NULL) ||
      (pStreamData->check != aSERIALCHECK))
    return aErrParam;
 
  written = write(pStreamData->nSerialDevice, pData, 1);
  if (written != 1)
    return(aErrIO);
  
  return(aErrNone);

} /* end of sSerialStreamPut */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSerialStreamDelete
 */

aErr sSerialStreamDelete(void* ref)
{
  aUnixSerialStream* pStreamData = (aUnixSerialStream*)ref;
  aErr err = aErrNone;

  /* check for valid stream data */
  if ((pStreamData == NULL) ||
      (pStreamData->check != aSERIALCHECK))
    return aErrParam;

  /* restore the default input buffer */
  if ((err == aErrNone) && (pStreamData->nSerialDevice != 0)) {
    
    /* restore old port settings */
    tcsetattr(pStreamData->nSerialDevice, TCSANOW, &pStreamData->oldtio);

    /* close serial device/port */
    close(pStreamData->nSerialDevice);
    
    /* invalidate the data */
    pStreamData->nSerialDevice = 0;
    memset(&pStreamData->oldtio, 0, sizeof(pStreamData->oldtio));
    memset(&pStreamData->oldtio, 0, sizeof(pStreamData->oldtio));
  }

  pStreamData->check = 0;
  
  aMemFree((aMemPtr)pStreamData);

  return(err);

} /* end of sSerialStreamDelete */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sStreamOpenSerial
 */

aErr sStreamOpenSerial(aUnixSerialStream* pStreamData)
{
  aErr err = aErrNone;
  char deviceName[200];

  if (pStreamData == NULL)
    err = aErrIO;

#define _POSIX_SOURCE 1 /* POSIX compliant source */
  
  /* open serial port/device to be non-blocking
   * (read returns immediately)
   */
  if (err == aErrNone) {
    aStringCopy(deviceName, "/dev/");
    aStringCat(deviceName, pStreamData->devName);
    pStreamData->nSerialDevice = open(deviceName, 
  				      O_RDWR 
  				      | O_NOCTTY 
  				      | O_NDELAY
  				      | O_NONBLOCK);
  
    /* one of the reasons it can fail is lack of privs... */
    if (pStreamData->nSerialDevice < 0) {
      int osErr = errno;
      switch (osErr) {
      case ENOENT:
        err = aErrNotFound;
        break;
      case EACCES:
        err = aErrPermission;
        break;
      default:
        err = aErrIO;
        break;
      } /* switch */
      pStreamData->nSerialDevice = 0;
    }
  }

  if (err == aErrNone) {
    if (fcntl(pStreamData->nSerialDevice, F_SETFL, 0)) {
      err = aErrIO;
    }
  }

  /* get the current options and save them for later reset */
  if (err == aErrNone) {
    if (tcgetattr(pStreamData->nSerialDevice, &pStreamData->oldtio))
      err = aErrIO;
    
    /* set the options we want */
    else {
      int speed = 0;

      pStreamData->newtio = pStreamData->oldtio;
      /* ignore break, parity */
      pStreamData->newtio.c_iflag = IGNBRK | IGNPAR;
      /* now output control */
      pStreamData->newtio.c_oflag = 0;
      /* basic control flags */
      pStreamData->newtio.c_cflag = CS8 | CLOCAL | CREAD;
      /* local modes turned off */
      pStreamData->newtio.c_lflag = 0;
      /* set up the speed */
      switch (pStreamData->nBaudRate) {
      case 2400: speed = B2400; break;
      case 4800: speed = B4800; break;
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      } /* switch */
      cfsetispeed(&pStreamData->newtio, speed);
      cfsetospeed(&pStreamData->newtio, speed);
#if 0
    this used to work on MACX
      pStreamData->newtio.c_ispeed = pStreamData->nBaudRate;
      pStreamData->newtio.c_ospeed = pStreamData->nBaudRate;
#endif
      pStreamData->newtio.c_cc[VMIN] = 0;
      pStreamData->newtio.c_cc[VTIME] = 0;
      if (tcsetattr(pStreamData->nSerialDevice, TCSANOW, 
      		    &pStreamData->newtio))
        err = aErrIO;
    }
  }

  /* flush any line noise gook */
  if (err == aErrNone) {
    if (tcflush(pStreamData->nSerialDevice, TCIFLUSH)) {
      err = aErrIO;
    }
  }

  return(err);

} /* end of sStreamOpenSerial routine */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateSerial
 */

aLIBRETURN aStream_CreateSerial(aIOLib ioRef, 
				const char *pPortName,
				const unsigned int baudRate, 
				aStreamRef* pStreamRef,
				aErr* pErr)
{
  aErr err = aErrNone;
  aStreamRef streamRef = NULL;
  aUnixSerialStream* pStreamData = NULL;


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  aVALIDIO(ioRef);
  if ((err == aErrNone) 
      && ((pPortName == NULL) || (pStreamRef == NULL)))
    err = aErrParam;
  else if (!((baudRate == 2400) ||
             (baudRate == 4800) ||
             (baudRate == 9600) ||
             (baudRate == 19200) ||
             (baudRate == 38400) ||
             (baudRate == 57600) ||
             (baudRate == 115200)))
    err = aErrRange;
 
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * initialize return values
   */

  if (err == aErrNone)
    *pStreamRef = NULL;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * create the stream
   */
  
  if (err == aErrNone) {
    pStreamData = (aUnixSerialStream*)aMemAlloc(
    				   sizeof(aUnixSerialStream));
    if (pStreamData == NULL)
      err = aErrMemory;
    else {
      aBZero(pStreamData, sizeof(aUnixSerialStream));
      pStreamData->nBaudRate = baudRate;
      aStringCopy(pStreamData->devName, pPortName);
      pStreamData->check = aSERIALCHECK;
    }
  }

  if (err == aErrNone)
    aStream_Create(ioRef, 
    		   sSerialStreamGet,
    		   sSerialStreamPut,
    		   sSerialStreamDelete,
    		   pStreamData, 
    		   &streamRef, 
    		   &err);

  /* try to open the serial port */
  if (err == aErrNone)
    err = sStreamOpenSerial(pStreamData);

  if (err == aErrNone) {
    *pStreamRef = streamRef;
  } else {
    if (streamRef)
      aStream_Destroy(ioRef, streamRef, NULL);
  }

  if (pErr != NULL)
    *pErr = err;

  return(err != aErrNone);

} /* aStream_CreateSerial */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSocketStreamPrepare
 */

aErr sSocketStreamPrepare(aUnixSocketStream* pStreamData)
{
  aErr err = aErrNone;

  /* for server sockets, see if we are still listening */
  if ((err == aErrNone)
      && (pStreamData->flags & aSOCKETLISTENING))
    err = sSocketListen(pStreamData);

  /* see if we have connected yet */
  if ((err == aErrNone)
      && (pStreamData->flags & aSOCKETNEEDSCONNECT)) {
    if (!pStreamData->client)
      pStreamData->client = socket(AF_INET, SOCK_STREAM, 0);
    if (pStreamData->client == -1) {
      SHOWERRNO("socket creation");
      err = aErrIO;
    }

    if (err == aErrNone) {
      if (connect(pStreamData->client,
    		(struct sockaddr *)&pStreamData->clientAddr,
    		sizeof(pStreamData->clientAddr)) == -1) {
        err = aErrEOF;
      } else {
        err = sSetSocketOptions(pStreamData->client);
        pStreamData->flags &= ~aSOCKETNEEDSCONNECT;
        pStreamData->flags |= aSOCKETOPEN;
      }
    }
  }

  return err;

} /* sSocketStreamPrepare */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSocketStreamGet
 */

aErr sSocketStreamGet(char* pData, void* ref)
{
  aErr err = aErrNone;
  aUnixSocketStream* pStreamData = (aUnixSocketStream*)ref;
  int avail = 0;

  /* check for valid stream data */
  if ((pStreamData == NULL) ||
      (pStreamData->check != aSOCKETCHECK))
    err = aErrParam;

  /* if we are still listening for a connection, do so */
  if (err == aErrNone)
    err = sSocketStreamPrepare(pStreamData);

  /* check to see if there are any bytes available */
  if ((err == aErrNone)
      && (ioctl(pStreamData->client, FIONREAD, &avail) == -1)) {
    SHOWERRNO("ioctl getting bytes available failed");
    err = aErrIO;
  }

  /* if none available, return not ready */
  if ((err == aErrNone) && (avail == 0))
    err = aErrNotReady;

  /* get the next byte */
  if ((err == aErrNone) 
       && (recv(pStreamData->client, pData, 1, 0) == -1)) {
    switch (errno) {
    case ENOTCONN: /* not connected */
      err = aErrNotReady;
      break;
    case EAGAIN: /* timeout */
      err = aErrEOF;
      break;
    default:
      SHOWERRNO("recv failed");
      err = aErrIO;
      break;
    }
  }

  return err;

} /* sSocketStreamGet */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSocketStreamPut
 */

aErr sSocketStreamPut(char* pData, void* ref)
{
  aErr err = aErrNone;
  aUnixSocketStream* pStreamData = (aUnixSocketStream*)ref;

  /* check for valid stream data */
  if ((pStreamData == NULL) ||
      (pStreamData->check != aSOCKETCHECK))
    err = aErrParam;

  /* if we are still listening for a connection, do so */
  if (err == aErrNone)
    err = sSocketStreamPrepare(pStreamData);

  /* send the next byte */
#ifndef MSG_NOSIGNAL
  if ((err == aErrNone)
      && (send(pStreamData->client, pData, 1, 0) == -1))
#else /* MSG_NOSIGNAL */
  if ((err == aErrNone)
      && (send(pStreamData->client, pData, 1, MSG_NOSIGNAL) == -1))
#endif /* MSG_NOSIGNAL */
  {
    if (errno == EPIPE)
      err = aErrConnection;
    else
      err = aErrIO;
  }

  return err;

} /* sSocketStreamPut */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSocketStreamWrite
 */

aErr sSocketStreamWrite(const char* pData, 
			const unsigned long nSize,
			void* ref)
{
  aErr err = aErrNone;
  aUnixSocketStream* pStreamData = (aUnixSocketStream*)ref;

  /* check for valid stream data */
  if ((pStreamData == NULL) ||
      (pStreamData->check != aSOCKETCHECK))
    err = aErrParam;

  /* if we are still listening for a connection, do so */
  if (err == aErrNone)
    err = sSocketStreamPrepare(pStreamData);

  /* send the bytes */
  if ((err == aErrNone)
      && (send(pStreamData->client, pData, (int)nSize, 0) == -1)) {
    SHOWERRNO("error sending");
    err = aErrIO;
  }

  return err;

} /* sSocketStreamWrite */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSocketStreamDelete
 */

aErr sSocketStreamDelete(void* ref)
{
  aErr err = aErrNone;
  aUnixSocketStream* pStreamData = (aUnixSocketStream*)ref;

  /* check for valid stream data */
  if ((pStreamData == NULL) ||
      (pStreamData->check != aSOCKETCHECK))
    err = aErrIO;

  if ((err == aErrNone)
       && (pStreamData->flags & aSOCKETOPEN)) {
    shutdown(pStreamData->client, 0x02);
    close(pStreamData->client);
    pStreamData->flags &= ~aSOCKETOPEN;
  }

  /* release any server portion */
  if (pStreamData->pServer) {
    err = sServerSocketRelease(pStreamData->pServer);
    pStreamData->pServer = NULL;
  }

  pStreamData->check = 0;
  aMemFree(pStreamData);

  return err;

} /* sSocketStreamDelete */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSocketListen
 */

aErr sSocketListen(aUnixSocketStream* pStreamData)
{
  if (pStreamData->flags & aSOCKETLISTENING) {
    socklen_t len = sizeof(pStreamData->clientAddr);
    pStreamData->client = 
         accept(pStreamData->pServer->socket,
		(struct sockaddr *)&pStreamData->clientAddr,
		&len);

    /* check for errors */
    if (pStreamData->client == -1) {
      switch (errno) {
      case EWOULDBLOCK:
        return aErrEOF;
      default:
        SHOWERRNO("accept failed");
        return aErrIO;
	break;
      } /* switch */
    } /* if bad client */

    /* adjust the flags to show that we are no longer
     * listening and we have an accepted connection */
    pStreamData->flags &= ~aSOCKETLISTENING;
    pStreamData->flags |= (aSOCKETACCEPTED | aSOCKETOPEN);
  }
  
  return aErrNone;

} /* sSocketListen */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sServerSocketAcquire
 */

aErr sServerSocketAcquire(
  aUnixIOLib* pIOLib,
  unsigned int address,
  unsigned short port,
  aUNIXIOSocket** ppServer
)
{
  aErr uiErr = aErrNone;
  aUNIXIOSocket* pServer;

  /* first, search the librarie's socket list to see if it is 
   * already there */
  pServer = pIOLib->pServerPortList;
  while (pServer) {
    if ((pServer->address == address)
        && (pServer->port == port)) 
      break;
    pServer = pServer->pNext;
  }

  /* if not already found, create it */
  if (pServer == NULL) {
    pServer = (aUNIXIOSocket*)aMemAlloc(sizeof(aUNIXIOSocket));
    if (pServer == NULL)
      uiErr = aErrMemory;
      
    if (uiErr == aErrNone) {
      aBZero(pServer, sizeof(aUNIXIOSocket));
      pServer->address = address;
      pServer->port = port;
      pServer->socket = socket(AF_INET, SOCK_STREAM, 0);
      if (pServer->socket == -1) {
        SHOWERRNO("creating socket");
        uiErr = aErrIO;
      }

      if (uiErr == aErrNone)
        uiErr = sSetSocketOptions(pServer->socket);

      /* set the socket to be non-blocking */
      if (uiErr == aErrNone) {
	int enable = 1;
    
	/* set the socket to non-blocking so we don't hang
	 * on accept calls if no client is available yet
	 */
	if (ioctl(pServer->socket, FIONBIO, &enable) == -1) {
	  SHOWERRNO("unable to ioctl socket for non-blocking");
	  uiErr = aErrIO;
	}
      }
    }

    /* bind to the address as a server */
    if (uiErr == aErrNone) {
      struct sockaddr_in addr;
      aBZero(&addr, sizeof(struct sockaddr_in));
      addr.sin_family = AF_INET;
      addr.sin_port = htons(port);
      addr.sin_addr.s_addr = htonl(address);
      if (bind(pServer->socket, (struct sockaddr *)&addr, 
	       sizeof(struct sockaddr_in))) {
        int tmpErr = errno;
        char msg[200];
	sprintf(msg, "unable to bind server socket %X:%d\n",
		address, port);
        SHOWERRNO(msg);
        tmpErr = aErrIO;
        shutdown(pServer->socket, 0x02);
	close(pServer->socket);
      }
    }

    /* establish how many connections we will allow and listen */
    if ((uiErr == aErrNone)
        && (listen(pServer->socket, aTCP_MAXCONNECT) == -1)) {
      SHOWERRNO("listening failed");
      uiErr = aErrIO;
    }

    /* link this in to the library's server port list */
    if (uiErr == aErrNone) {
      pServer->pNext = pIOLib->pServerPortList;
      pIOLib->pServerPortList = pServer;
    }
    
    if (uiErr != aErrNone)
      aMemFree((aMemPtr)pServer);

  } /* if creating server */
  
  if (uiErr == aErrNone) {
    pServer->count++;
    pServer->vpUnixIOLib = (void*)pIOLib;
    *ppServer = pServer;
  }

  return uiErr;

} /* sServerSocketAcquire */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sServerSocketRelease
 */

aErr sServerSocketRelease(aUNIXIOSocket* pServer)
{
  aErr err = aErrNone;
  aUnixIOLib* pIOLib;
  aUNIXIOSocket* pPrev = NULL;
  aUNIXIOSocket* pTemp;

  aAssert(pServer);
  aAssert(pServer->vpUnixIOLib);

  pIOLib = (aUnixIOLib*)pServer->vpUnixIOLib;
  
  /* first, find the server in the list */
  pTemp = pIOLib->pServerPortList;
  while (pTemp) {
    if (pTemp == pServer)
      break;
    pPrev = pTemp;
    pTemp = pTemp->pNext;
  }

  if (pTemp == NULL)
    err = aErrParam;
    
  if (err == aErrNone) {
    /* decrement the count */
    if (pTemp->count > 0)
      pTemp->count--;
    
    /* if no more users, free the record */
    if (pTemp->count == 0) {
    
      /* unlink from the list */
      if (pPrev == NULL)
        pIOLib->pServerPortList = pTemp->pNext;
      else
        pPrev->pNext = pTemp->pNext;
    
      shutdown(pTemp->socket, 0x02);
      close(pTemp->socket);
      aMemFree(pTemp);
    }
  }

  return err;

} /* sServerSocketRelease */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sSetSocketOptions
 */

aErr sSetSocketOptions(
  const int iSocket
)
{
  aErr err = aErrNone;
  int nodelay = 1;
  struct protoent* pProto = getprotobyname("TCP");

  /* NODELAY prevents byte coelescing */
  aAssert(pProto);
  if(setsockopt(iSocket, pProto->p_proto, TCP_NODELAY,
	        (char*)&nodelay, sizeof(nodelay))) {
    SHOWERRNO("setsockopt TCP_NODELAY failed");
    err = aErrIO;
  }

  /* SO_RCVTIMEO is set to return immediately */
  if (err == aErrNone) {
    struct timeval timeStruct;
    timeStruct.tv_sec = 0;
    timeStruct.tv_usec = 0;
    if (setsockopt(iSocket, SOL_SOCKET, SO_RCVTIMEO,
	           (char*)&timeStruct, sizeof(timeStruct))) {
      SHOWERRNO("setsockopt SO_RCVTIMEO failed");
      err = aErrIO;
    }
  }

  /* DONTLINGER kills the socket immediately on close */
  if (err == aErrNone) {
    struct linger lingerStruct;
    lingerStruct.l_onoff = 1;
    lingerStruct.l_linger = 0;
    if (setsockopt(iSocket, SOL_SOCKET, SO_LINGER,
	           (char*)&lingerStruct, sizeof(lingerStruct))) {
      SHOWERRNO("setsockopt SO_LINGER failed");
      err = aErrIO;
    }
  }

  /* SO_NOSIGPIPE causes send to return EPIPE rather than signalling
   * a SIGPIPE */
#ifndef MSG_NOSIGNAL
  {
    int noSigPipe = 1;
    /* NOSIGPIPE returns EPIPE instead of signalling on BSD only */
    if (setsockopt(iSocket, SOL_SOCKET, SO_NOSIGPIPE,
	        (char*)&noSigPipe, sizeof(noSigPipe))) {
      SHOWERRNO("setsockopt SO_NOSIGPIPE failed");
      err = aErrIO;
    }
  }
#endif /* MSG_NOSIGNAL */

  return err;

} /* sSetSocketOptions */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_CreateSocket
 */

aLIBRETURN aStream_CreateSocket(
  aIOLib ioRef, 
  const unsigned long address, 
  const unsigned short port, 
  const aBool bServer,
  aStreamRef* pStreamRef, 
  aErr* pErr
)
{
  aErr err = aErrNone;
  aUnixIOLib* pIOLib = (aUnixIOLib*)ioRef;
  aUnixSocketStream* pStreamData = NULL;
  aStreamRef streamRef = NULL;

  
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * check params
   */
  aVALIDIO(ioRef);
  if (pStreamRef == NULL)
    err = aErrParam;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * initialize return values
   */
  if (err == aErrNone)
    *pStreamRef = NULL;

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   * create the stream
   */  
  if (err == aErrNone) {
    pStreamData = (aUnixSocketStream*)aMemAlloc(
    				   sizeof(aUnixSocketStream));
    if (pStreamData == NULL)
      err = aErrMemory;
    else {
      aBZero(pStreamData, sizeof(aUnixSocketStream));
      pStreamData->pLib = (aUnixIOLib*)ioRef;

      /* start with all zeros */
      aBZero(pStreamData, sizeof(aUnixSocketStream));

      if (bServer == aTrue) {
        err = sServerSocketAcquire(pIOLib, address,
				   port, &pStreamData->pServer);
        if (err == aErrNone)
          pStreamData->flags |= aSOCKETLISTENING;
        else
          aMemFree((aMemPtr)pStreamData);
      } else {
        /* set the client address */
	pStreamData->clientAddr.sin_family = AF_INET;
	pStreamData->clientAddr.sin_port = htons(port);
	pStreamData->clientAddr.sin_addr.s_addr = htonl(address);
        pStreamData->flags |= aSOCKETNEEDSCONNECT;
      }
      
      /* set up the validation check number */ 
      if (err == aErrNone)
        pStreamData->check = aSOCKETCHECK;
    }
  }

  /* create the stream object */
  if (err == aErrNone)
    aStream_Create(ioRef, 
    		   sSocketStreamGet,
    		   sSocketStreamPut,
    		   sSocketStreamDelete,
    		   pStreamData, 
    		   &streamRef, 
    		   &err);

  if (err == aErrNone) {
    /* setting sSocketStreamWrite allows bulk writes without
     * calling the put proc once for each byte.  This is much
     * more efficient allowing BrainStem packets to be sent
     * in one TCP/IP packet rather than each byte in a separate
     * TCP/IP packet.  This is because we have disabled byte
     * coelescing on the socket.
     */
    aStream* pStream = (aStream*)streamRef;
    pStream->writeProc = sSocketStreamWrite;
    *pStreamRef = streamRef;
  } else {
    if (streamRef)
      aStream_Destroy(ioRef, streamRef, NULL);
  }

  if (pErr != NULL)
    *pErr = err;

  return(err != aErrNone);

} /* aStream_CreateSocket */

#endif /* aUNIX */

