/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemTests.c	                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: stdio testing module for the aStem shared library. */
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

#include "aCmd.tea"

#include "aStemTests.h"
#include "aStemText.h"

typedef struct aTEAFileDebug {
  void*			pStem;
  char			packetBuffer[aSTEMMAXPACKETBYTES];
  int			nBytesInBuffer;
  unsigned char		checksum;
  int			nBytesSent;
  aBool			bDone;
  unsigned char		address;

} aTEAFileDebug;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static void aStream_WriteString(aStreamRef output,
				char* msg);
static aBool aStemHBScanner(const unsigned char module,
			    const unsigned char dataLength,
			    const char* data,
			    void* ref);
static aBool sRelayUnidirectionalTest(aIOLib ioRef,
			              aStemLib stemRef,
							   		  aStreamRef s1, 
							          aStreamRef s2, 
							          const int bufferSize, 
							          const int duration,
							          aStreamRef out);
static aBool sRelayBidirectionalTest(aIOLib ioRef,
			       aStemLib stemRef,
							   		 aStreamRef s1, 
							         aStreamRef s2, 
							         const int bufferSize, 
							         const int duration,
							         aStreamRef out);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStream_WriteString
 */

void aStream_WriteString(aStreamRef output, char* msg)
{
  aStream_Write(aStreamLibRef(output), output, msg, 
  			    aStringLen(msg), NULL);

} /* aStream_WriteString */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemHBScanner
 */

aBool aStemHBScanner(const unsigned char module,
		     const unsigned char dataLength,
		     const char* data,
		     void* ref)
{
  if ((dataLength == 2) && (data[0] == cmdHB)) {
    unsigned char* p = (unsigned char*)ref;
    *p = (unsigned char)(*p + 1);
  }

  return aTrue;

} /* aStemHBScanner */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemTests
 */

aBool aStemTests(aIOLib ioRef, aStreamRef out)
{
  char msg[200];
  aErr stemErr;
  aStemLib stemRef;

  aStream_WriteLine(ioRef, out, " Starting Stem tests", NULL);

  /* aStem_GetLibRef */

  aStringCopy(msg, "  GetLibRef with null params...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aStem_GetLibRef(NULL, NULL))
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  aStringCopy(msg, "  GetLibRef with null lib ref");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aStem_GetLibRef(NULL, &stemErr))
    return 1;
  aStringCopy(msg, ", checking err code...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (stemErr != aErrParam)
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  aStringCopy(msg, "  GetLibRef with null error param");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (aStem_GetLibRef(&stemRef, NULL))
    return 1;
  aStringCopy(msg, ", closing...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (aStem_ReleaseLibRef(stemRef, NULL))
    return 1;
  stemRef = NULL;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* aStem_ReleaseLibRef */

  aStringCopy(msg, "  ReleaseLibRef with null params...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aStem_GetLibRef(NULL, NULL))
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  aStringCopy(msg, "  ReleaseLibRef with null lib ref");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aStem_ReleaseLibRef(NULL, &stemErr))
    return 1;
  aStringCopy(msg, ", checking err code...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (stemErr != aErrParam)
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* aStem_DebugLine */

#ifndef aPALM
  aStringCopy(msg, "  DebugLine null params...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aStem_DebugLine(NULL, NULL, NULL))
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  aStringCopy(msg, "  DebugLine null lib...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aStem_DebugLine(NULL, "test line", &stemErr))
    return 1;
  aStringCopy(msg, ", checking err code...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (stemErr != aErrParam)
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);
#endif /* aPALM */

  if (aStem_GetLibRef(&stemRef, NULL))
    return 1;

  aStringCopy(msg, "  DebugLine null text...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aStem_DebugLine(stemRef, NULL, &stemErr))
    return 1;
  aStringCopy(msg, ", checking err code...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (stemErr != aErrParam)
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  if (aStem_ReleaseLibRef(stemRef, NULL))
    return 1;
  stemRef = NULL;

 
  /* open up the library for some tests */
  aStem_GetLibRef(&stemRef, NULL);


  /* aPacket_Create */

#ifndef aPALM
  aStringCopy(msg, "  Packet Build with null params...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aPacket_Create(NULL, 0, 0, 0, NULL, NULL))
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);
#endif /* aPALM */

  aStringCopy(msg, "  Packet Build with null params...");
  aStream_Write(ioRef, out, msg, aStringLen(msg), NULL);
  if (!aPacket_Create(stemRef, 0, 0, 0, NULL, NULL))
    return 1;
  aStream_WriteLine(ioRef, out, "passed", NULL);


  /* close up the library, we are done */
  aStem_ReleaseLibRef(stemRef, NULL);
  stemRef = NULL;


  aStream_WriteLine(ioRef, out, " Stem tests complete", NULL);

  return aFalse;

} /* aStemTests */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemLiveTests
 */
aBool aStemLiveTests(aIOLib ioRef, aStreamRef out)
{
  aStemLib stemRef;
  aStreamRef serialStream;
#ifdef aWIN
  char* portname = "COM1";
#endif /* aWIN */
#ifdef aWINCE
  char* portname = "COM1:";
#endif /* aWINCE */
#ifdef aPALM
  char* portname = "serial";
#endif /* aPALM */
#ifdef aMAC
  char* portname = "printer";
#endif /* aMAC */
#ifdef aUNIX
#ifdef aMACX
  char* portname = "tty.USA28X111P1.1";
#else /* aMACX */
  char* portname = "ttyS0";
#endif /* aMACX */
#endif /* aUNIX */ 

  aErr stemErr = aErrNone;
  unsigned char i;
  aPacketRef packet;
  unsigned long start, end;

  aStream_WriteLine(ioRef, out, " Starting Live Stem tests", NULL);

  /* open up the library for some tests */
  aStream_WriteString(out, "  Opening Stem Libarary...");
  if (aStem_GetLibRef(&stemRef, NULL))
    return aTrue;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* create a stream connection to the serial port */
  aStream_WriteString(out, "  Creating Serial Stream...");
  if (aStream_CreateSerial(ioRef, portname, 9600, &serialStream, NULL))
    return aTrue;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* set the serial stream for the stem library */
  aStream_WriteString(out, "  Setting Serial Stream...");
  if (aStem_SetStream(stemRef, serialStream, kStemModuleStream, NULL))
    return aTrue;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* first, get a heartbeat going by looking for a packet */
  aStream_WriteString(out, "  Establishing a Heartbeat...");
  aIO_GetMSTicks(ioRef, &start, NULL);
  end = start;
  while (end < start + 5000) {
    if (!aStem_GetPacket(stemRef, NULL, NULL, 500, &packet, NULL))
      aPacket_Destroy(stemRef, packet, NULL);
    aIO_GetMSTicks(ioRef, &end, NULL);
  }
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* try sending some debug packets to check bi-directional comm */
  aStream_WriteString(out, "  Sending Debug Packets...");
  for (i = 1; i < 5; i++) {
    unsigned char replyAddr;
    unsigned char replyLen;
    int j;
    char data[5] ;
    char reply[5];

    data[0] = cmdDEBUG;
    data[1] = 'd';
    data[2] = 'a';
    data[3] = 't';
    data[4] = 'a';

    aStem_DebugLine(stemRef, "creating", NULL);
    aPacket_Create(stemRef,
		   2,
		   i,
		   data,
		   &packet,
		   &stemErr);
    if (stemErr != aErrNone)
      return aTrue;

/*    aStem_DebugLine(stemRef, "sending", NULL); */
    aStem_SendPacket(stemRef,
		     packet,
		     &stemErr);
    if (stemErr != aErrNone)
      return aTrue;

/*    aStem_DebugLine(stemRef, "getting", NULL); */
    aStem_GetPacket(stemRef, 
		    NULL, 
		    NULL, 
    		    1000, 
		    &packet, 
		    &stemErr);
    if (stemErr != aErrNone)
      return aTrue;

/*    aStem_DebugLine(stemRef, "unwrapping", NULL); */
    aPacket_GetData(stemRef,
		    packet,
		    &replyAddr,
		    &replyLen,
		    reply,
		    &stemErr);
    if (stemErr != aErrNone)
      return aTrue;

    /* compare the length and address */
/*    aStem_DebugLine(stemRef, "comparing", NULL); */
    if ((replyAddr != 2)
        || (replyLen != i))
      return aTrue;

    /* compare the data contents */
    for (j = 0; j < i; j++)
      if (reply[j] != data[j])
        return aTrue;

    /* clean up the packet */
/*    aStem_DebugLine(stemRef, "cleaning", NULL); */
    aPacket_Destroy(stemRef, packet, &stemErr);
    if (stemErr != aErrNone)
      return aTrue;
  }
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* just get some heartbeats and bail */
  aStream_WriteString(out, "  Relaying Heartbeats");
  i = 0;
  
  /* loop for 10 seconds and see if we accumulate the heartbeats */
  aIO_GetMSTicks(ioRef, &start, NULL);
  do {
    aErr getErr;
    aStem_GetPacket(stemRef, aStemHBScanner, &i,
    		    1000, &packet, &getErr);
    if (getErr == aErrNone)
      aPacket_Destroy(stemRef, packet, NULL);
    aIO_GetMSTicks(ioRef, &end, NULL);
    aStream_WriteString(out, ".");
  } while ((i < 9) && (start + 10000 > end));
  if (start + 10000 <= end)
    return aTrue;
  if (stemErr != aErrNone)
    return aTrue;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  /* hammer the aStem TEA file IO utilities */
  if (aStemTEAFileTests(ioRef, stemRef, out, 2)) 
    aStream_WriteLine(ioRef, out, "!!! FAILED !!!", NULL);

  /* close up the library, we are done */
  aStream_WriteString(out, "  Closing Stem Libarary...");
  if (aStem_ReleaseLibRef(stemRef, NULL))
    return aTrue;
  aStream_WriteLine(ioRef, out, "passed", NULL);

  aStream_WriteLine(ioRef, out, " Live Stem tests complete", NULL);

  return aFalse;

} /* aStemLiveTests */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRelayUnidirectionalTest
 */
aBool sRelayUnidirectionalTest(aIOLib ioRef,
			       aStemLib stemRef,
			       aStreamRef s1, 
			       aStreamRef s2, 
			       const int bufferSize, 
			       const int duration,
			       aStreamRef output)
{
  aErr err;
  unsigned long now;
  unsigned long done;
  int count = 1;
  int out = 0;
  unsigned char outChar = 0;
  unsigned char inChar = 0;
  unsigned char in;
  char* pData;
  int i;
  int total = 0;
  char num[20];
  aPacketRef packet;

  pData = (char*)aMemAlloc((aMemSize)bufferSize);
  if (!pData)
    return aTrue;

  if (aIO_GetMSTicks(ioRef, &now, &err))
    return aTrue;

  done = now + (duration * 1000);

  /* just loop until the time is up */
  while (now < done) {

    /* reset the size if we are done */
    if (count >= bufferSize)
      count = 1;

    /* if there is room to write, stuff some bytes out */
    if (out < bufferSize) {
      for (i = 0; i < count; i++)
        pData[i] = (char)outChar++;
      if (aStream_Write(ioRef, s1, pData, (aMemSize)count, &err))
        return aTrue;
      out += count;
      count++;
    }

    /* if there are bytes out, see if they came back in */
    if (out > 0) {
      aStream_Read(ioRef, s2, (char*)&in, 1, &err);
      if (err == aErrNone) {
        if (in != inChar) {
          printf("Error: expected 0x%X, got 0x%X\n", inChar, in);
          return aTrue;
        }
        inChar++;
        out--;
        total++;
      } else if (err != aErrNotReady)
        return aTrue;
    }

    if (!aStem_GetPacket(stemRef, NULL, NULL, 0, &packet, NULL)) {
      char errorStr[100];
      
      aStream_WriteLine(aStreamLibRef(output), output, 
  			" ERROR!!", NULL);
      aPacket_Format(stemRef, packet, errorStr, 100, NULL);
      aStream_WriteString(output, "    unexpected packet: ");
      aStream_WriteLine(aStreamLibRef(output), output,
      			errorStr, NULL);
      return aTrue;
    }

    if (aIO_GetMSTicks(ioRef, &now, &err))
      return aTrue;
  }

  aStringFromInt(num, total);
  total /= duration;

  aStream_WriteString(output, "    throughput = ");
  aStream_WriteString(output, num);
  aStream_WriteLine(aStreamLibRef(output), output, 
  				    " bytes per second", NULL);

  aMemFree(pData);

  /* clean any data out of the pipe that is left */
  while (out) {
    aStream_Read(ioRef, s2, (char*)&in, 1, &err);
    if (err == aErrNone)
      out--;
    else if (err != aErrNotReady)
      return aTrue;
  }

  return aFalse;

} /* sRelayUnidirectionalTest */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRelayBidirectionalTest
 */
aBool sRelayBidirectionalTest(aIOLib ioRef,
			      aStemLib stemRef,
			      aStreamRef s1, 
			      aStreamRef s2, 
			      const int bufferSize, 
			      const int duration,
			      aStreamRef output)
{
  aErr err;
  unsigned long now;
  unsigned long done;
  int count1 = 1;
  int count2 = 1;
  int out1 = 0;
  int out2 = 0;
  unsigned char out1Char = 0;
  unsigned char in1Char = 0;
  unsigned char out2Char = 0;
  unsigned char in2Char = 0;
  unsigned char in;
  char* pData;
  int i;
  int total = 0;
  char num[20];
  aPacketRef packet;

  pData = (char*)aMemAlloc((aMemSize)bufferSize);
  if (!pData)
    return aTrue;

  if (aIO_GetMSTicks(ioRef, &now, &err))
    return aTrue;

  done = now + (duration * 1000);

  /* just loop until the time is up */
  while (now < done) {

    /* reset the size if we are done */
    if (count1 >= bufferSize)
      count1 = 1;
    if (count2 >= bufferSize)
      count2 = 1;

    /* if there is room to write, stuff some bytes out */
    if (out1 < bufferSize) {
      for (i = 0; i < count1; i++)
        pData[i] = (char)out1Char++;
      if (aStream_Write(ioRef, s1, pData, (aMemSize)count1, &err))
        return aTrue;
      out1 += count1;
      count1++;
    }

    /* if there is room to write, stuff some bytes out */
    if (out2 < bufferSize) {
      for (i = 0; i < count2; i++)
        pData[i] = (char)out2Char++;
      if (aStream_Write(ioRef, s2, pData, (aMemSize)count2, &err))
        return aTrue;
      out2 += count2;
      count2++;
    }
    
    /* if there are bytes out, see if they came back in */
    if (out1 > 0) {
      aStream_Read(ioRef, s2, (char*)&in, 1, &err);
      if (err == aErrNone) {
        if (in != in1Char)
          return aTrue;
        in1Char++;
        out1--;
        total++;
      } else if (err != aErrNotReady)
        return aTrue;
    }
    
    /* if there are bytes out, see if they came back in */
    if (out2 > 0) {
      aStream_Read(ioRef, s1, (char*)&in, 1, &err);
      if (err == aErrNone) {
        if (in != in2Char)
          return aTrue;
        in2Char++;
        out2--;
        total++;
      } else if (err != aErrNotReady)
        return aTrue;
    }

    if (!aStem_GetPacket(stemRef, NULL, NULL, 0, &packet, NULL)) {
      char errorStr[100];
      
      aStream_WriteLine(aStreamLibRef(output), output, 
  			" ERROR!!", NULL);
      aPacket_Format(stemRef, packet, errorStr, 100, NULL);
      aStream_WriteString(output, "    unexpected packet: ");
      aStream_WriteLine(aStreamLibRef(output), output,
      			errorStr, NULL);
      return aTrue;
    }

    if (aIO_GetMSTicks(ioRef, &now, &err))
      return aTrue;
  }

  aStringFromInt(num, total);
  total /= duration;

  aStream_WriteString(output, "    throughput = ");
  aStream_WriteString(output, num);
  aStream_WriteLine(aStreamLibRef(output), output, 
  				    " bytes per second", NULL);

  aMemFree(pData);

  /* clean any data out of the pipe that is left */
  while (out1) {
    aStream_Read(ioRef, s2, (char*)&in, 1, &err);
    if (err == aErrNone)
      out1--;
    else if (err != aErrNotReady)
      return aTrue;
  }
  while (out2) {
    aStream_Read(ioRef, s1, (char*)&in, 1, &err);
    if (err == aErrNone)
      out2--;
    else if (err != aErrNotReady)
      return aTrue;
  }

  return aFalse;

} /* sRelayBidirectionalTest */



#define PORTNAME1	"COM1"
#define PORTNAME2	"COM2"
#define BAUDRATE	115200 
#define RELAYMODULE	4
#define TESTSTEPSECONDS	600

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemRelayTests
 */
aBool aStemRelayTests(aIOLib ioRef, aStreamRef out)
{
  aStreamRef s1, s2;
#ifndef WIRED
  aStemLib stem;
#endif /* WIRED */

  aStream_WriteLine(aStreamLibRef(out), out, 
  			        " Starting Serial Relay Tests", NULL);

  aStream_WriteString(out, "  building streams...");
  if (aStream_CreateSerial(ioRef, PORTNAME1, BAUDRATE, &s1, NULL))
    return aTrue;
#ifndef WIRED
  /* in the non-wired case, construct a stem interface and get 
   * the serial relay channel from it */
  if (aStem_GetLibRef(&stem, NULL))
    return aTrue;
  if (aStem_SetStream(stem, s1, kStemModuleStream, NULL))
    return aTrue;
  if (aStem_CreateRelayStream(stem, RELAYMODULE, &s1, NULL))
    return aTrue;  			      
#endif /* WIRED */


  if (aStream_CreateSerial(ioRef, PORTNAME2, BAUDRATE, &s2, NULL))
    return aTrue;
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "passed", NULL);


#if 1
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "  testing in one direction single bytes", NULL);
  if (sRelayUnidirectionalTest(ioRef, stem, s1, s2, 1, 
  			       TESTSTEPSECONDS, out))
    return aTrue;
#endif

#if 1
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "  testing in one direction block writes", NULL);
  if (sRelayUnidirectionalTest(ioRef, stem, s1, s2, 32, 
  			       TESTSTEPSECONDS, out))
    return aTrue;
#endif

#if 1
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "  testing in other direction single bytes", NULL);
  if (sRelayUnidirectionalTest(ioRef, stem, s2, s1, 1, 
  			       TESTSTEPSECONDS, out))
    return aTrue;
#endif

#if 1
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "  testing in other direction block writes", NULL);
  if (sRelayUnidirectionalTest(ioRef, stem, s2, s1, 32, 
  			       TESTSTEPSECONDS, out))
    return aTrue;
#endif

#if 1
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "  testing bi-directional byte writes", NULL);
  if (sRelayBidirectionalTest(ioRef, stem, s2, s1, 1, 
  			      TESTSTEPSECONDS, out))
    return aTrue;
#endif

#if 1
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "  testing bi-directional block writes", NULL);
  if (sRelayBidirectionalTest(ioRef, stem, s2, s1, 32, 
  			      TESTSTEPSECONDS, out))
    return aTrue;
#endif

  aStream_WriteString(out, "  cleaning up streams...");
  if (aStream_Destroy(ioRef, s1, NULL))
    return aTrue;
  if (aStream_Destroy(ioRef, s2, NULL))
    return aTrue;
#ifndef WIRED
  if (aStem_ReleaseLibRef(stem, NULL))
    return aTrue;
#endif /* WIRED */
  aStream_WriteLine(aStreamLibRef(out), out, 
  				    "passed", NULL);

  aStream_WriteLine(aStreamLibRef(out), out, 
  				    " Serial Relay Tests Finished", NULL);
  
  return aFalse;
  
} /* aStemRelayTests */



aBool aStemTEAFileTestSingle(aIOLib ioRef, 
			     aStemLib stemRef,
                             aStreamRef out, 
                             unsigned char stemAddr,
                             int nFile, 
                             int nFileSize, 
                             char* pLabel)
{
  aErr err;
  aStreamRef teaOutput = NULL;
  aStreamRef teaInput = NULL;
  char buff[1200];
  char cuff[1200];
  unsigned int nSize;
  int i;

  /* create a null-terminated file */
  for (i = 0; i<nFileSize; i++) {
    buff[i]=(char)(66+(i%23));
    cuff[i]='\0';
  }
  buff[nFileSize] = '\0';
  cuff[nFileSize] = '\0';
  nSize = (unsigned int)(aStringLen(buff) + 1);

  aStream_WriteString(out, "  ");
  aStream_WriteString(out, pLabel);
  aStream_WriteString(out, " Output Stream... ");
  if (aStem_CreateTEAFileOutput(stemRef, stemAddr, nFile, &teaOutput, &err)) {
    aStream_WriteLine(ioRef, out, "*** OUTPUT OPEN FAILURE ***", NULL);
    return aTrue;
  }
  if (aStream_Write(ioRef, teaOutput, buff, nSize, &err)) {
    aStream_WriteLine(ioRef, out, "*** WRITE FAILURE ***", NULL);
    return aTrue;
  }

  /* stall to let the Stem save and checksum the file */
  {
    unsigned long start, end;
    aPacketRef packet;
    /* first, get a heartbeat going by looking for a packet */
    aIO_GetMSTicks(ioRef, &start, NULL);
    end = start;
    while (end < start + 2000) {
      if (!aStem_GetPacket(stemRef, NULL, NULL, 500, &packet, NULL))
        aPacket_Destroy(stemRef, packet, NULL);
      aIO_GetMSTicks(ioRef, &end, NULL);
    }
  }

  if (aStream_Destroy(ioRef, teaOutput, NULL)) {
    aStream_WriteLine(ioRef, out, "*** WRITE CHECKSUM ERROR ***", NULL);
    return aTrue;
  }
  aStream_WriteLine(ioRef, out, "success", NULL);

  aStream_WriteString(out, "  ");
  aStream_WriteString(out, pLabel);
  aStream_WriteString(out, " Input Stream... ");
  if (aStem_CreateTEAFileInput(stemRef, stemAddr, nFile, &teaInput, &err)) {
    aStream_WriteLine(ioRef, out, "*** INPUT OPEN FAILURE ***", NULL);
    return aTrue;
  }
  aStream_Read(ioRef, teaInput, cuff, nSize, &err);
  if (aStringCompare(buff,cuff) != 0) {
    aStream_WriteLine(ioRef, out, "*** READ DATA MISMATCH ***", NULL);
    return aTrue;
  }
  if (aStream_Destroy(ioRef, teaInput, NULL)) {
    aStream_WriteLine(ioRef, out, "*** READ CHECKSUM ERROR ***", NULL);
    return aTrue;
  }
  aStream_WriteLine(ioRef, out, "success", NULL);

  return aFalse;

} /* aStemTEAFileTestSingle */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemTEAFileTests
 */
aBool aStemTEAFileTests(aIOLib ioRef, aStemLib stemRef,
                        aStreamRef out, unsigned char stemAddr)
{  
  int n;
  char sLabel[16];
    
  aStream_WriteLine(ioRef, out, " Starting Live Stem TEA File tests", NULL);

  for (n = 0; n < 8; n++) {
    aStringCopy(sLabel,"  n    4x");
    sLabel[2]=(char)(48+n);
    if (aStemTEAFileTestSingle(ioRef, stemRef, out, stemAddr, n, 4, sLabel)) 
      return aTrue;
    aStringCopy(sLabel,"  n   61x");
    sLabel[2]=(char)(48+n);
    if (aStemTEAFileTestSingle(ioRef, stemRef, out, stemAddr, n, 61, sLabel)) 
      return aTrue;
    aStringCopy(sLabel,"  n  766x");
    sLabel[2]=(char)(48+n);
    if (aStemTEAFileTestSingle(ioRef, stemRef, out, stemAddr, n, 766, sLabel)) 
      return aTrue;
  }

  aStream_WriteLine(ioRef, out, " Live Stem TEA File tests complete", NULL);

  return aFalse;

} /* aStemTEAFileTests */
