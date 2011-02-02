/*                                                                 */
/* file: xxxxxxxx.c                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Simple program to demonstrate using the BrainStem  */
/* Moto using the C libraries.                                 */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2007. Acroname Inc.                              */
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

#include <aCommon/aStem.h>    /* stem library header   */
#include <aCommon/aMotion.h> /* motion control library */
#include <stdlib.h>

/* define which BrainStem module and servo we are talking to and 
 * how. In this case, we assume a GP 1.0 module 
 */
#define aMODULE    4
#define aCHANNEL 0

/* the portname below will depend entirely on your computer&#39;s      */
/* serial port configuration.  We use the command                  */
/*   ls /dev/tty.*                                                 */
/* to learn what the name of the serial adapter is on our machines */
/* Note: This value will change if you plug the adapter in other   */
/* USB ports, depending on your adapter type.                      */
#define aPORTNAME "acroname"
#define aPORTSPEED 9600

/* Program specific macros */
#define LOOPDELAY 250
#define SETMAX 200
#define SETINC 10

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* the main (and only) routine */
int main(int argc, char* argv[]) {
  aErr error = aErrNone;
  aIOLib ioLib;
  aStemLib stemLib;
  aStreamRef linkStream;
  short setpoint = 0;
  short pidval = 0;
  
  printf("Running the motor example.\n");     
  
  /* Get the references to the aIO and aStem library objects. */
  if (error == aErrNone)
    aIO_GetLibRef(&ioLib, &error);
  if (error == aErrNone)
    aStem_GetLibRef(&stemLib, &error);

  if( error != aErrNone )
    printf("Problem getting librefs\n");

  
  /* Build a link stream to communicate serially with the stem. */
  if (error == aErrNone)
    aStream_CreateSerial(ioLib, 
                         aPORTNAME, 
                         aPORTSPEED,                  
                         &linkStream, 
                         &error);

  if( error != aErrNone ) {
    printf("Problem forming link to Moto... ");
    
    switch(error) {
    case aErrNotFound:
      printf("port not found!\n");
      break;
    case aErrRange:
      printf("baud rate not available\n");
      break;
    case aErrOverrun:
      printf("buffer overrun\n");
    default:
      printf("unknown problem\n");
    };

    return -1;
  }

  if (error == aErrNone)
    aStem_SetStream(stemLib, 
                    linkStream, 
                    kStemModuleStream, 
                    &error);

  aPacketRef send_packet_ref, ret_packet_ref;

#define		cmdVAL_GET		17
#define		cmdVAL_SET		18
#define		cmdVAL_SAV		19

  int got_reply = 0;


  unsigned char get_baud_rate[2] = {cmdVAL_GET, 4};
  unsigned char set_baud_rate[3] = {cmdVAL_SET, 4, 4};
  unsigned char save_baud_rate[1] = {cmdVAL_SAV};

  while( got_reply < 2) {
  
  if( aPacket_Create(stemLib,
                     aMODULE,
                     2,
                     get_baud_rate,
                     &send_packet_ref,
                     &error) != 0 ) {
    printf("problem creating packet\n");
    return -1;
  }

    if( aStem_SendPacket(stemLib,
                         send_packet_ref,
                         &error) != 0 ) {
      printf("problem sending packet\n");
      return -1;
    }
    
    if ( aStem_GetPacket(stemLib,
                         NULL,
                         NULL,
                         500,
                         &ret_packet_ref,
                         &error) != 0 ) {
      printf("problem receiving packet\n");
      switch(error) {
      case aErrNotFound:
        printf("no packet available\n");
        break;
      case aErrIO:
        printf("no link set\n");
        break;
      case aErrTimeout:
        printf("Timeout\n");
        break;
      default:
        printf("unknown error\n");
      }
    }
    else {
      got_reply++;

      unsigned char address;
      unsigned char length;
      char data[aSTEMMAXPACKETBYTES];
      if( aPacket_GetData(stemLib,
                          ret_packet_ref,
                          &address,
                          &length,
                          data,
                          &error) != 0 ) {
        printf("problem getting packet data\n");
        return -1;
      }
      
      printf("Address: %d\n", address);
      unsigned char i = 0;
      for(i = 0; i < length; ++i) {
        printf("%d ", data[i]);
      }
      printf("\n");
      
      aPacket_Destroy(stemLib,
                      ret_packet_ref,
                      &error);
    }
  }
  
  if( aPacket_Create(stemLib,
                     aMODULE,
                     3,
                     set_baud_rate,
                     &send_packet_ref,
                     &error) != 0 ) {
    printf("problem creating packet\n");
    return -1;
  }
  
  if( aStem_SendPacket(stemLib,
                       send_packet_ref,
                       &error) != 0 ) {
    printf("problem sending packet\n");
    return -1;
  }
    
  if( aPacket_Create(stemLib,
                     aMODULE,
                     1,
                     save_baud_rate,
                     &send_packet_ref,
                     &error) != 0 ) {
    printf("problem creating packet\n");
    return -1;
  }
  
  if( aStem_SendPacket(stemLib,
                       send_packet_ref,
                       &error) != 0 ) {
    printf("problem sending packet\n");
    return -1;
  }
    

  /* release the libraries now that we are done whether there
   * were errors or not 
   */
  aStem_ReleaseLibRef(stemLib, NULL);
  aIO_ReleaseLibRef(ioLib, NULL);
  
  return error;
  
} /* main */
