/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aConsole_VMPorts.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a platform-independent Console   */
/*		command handlers.				   */
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

#include "aIOPorts.tea"
#include "aCmd.tea"

#include "aUtil.h"
#include "aConsole.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aConsole_VMPortCB
 */

aErr aConsole_VMPortCB(
  aTEAProcessID pid,
  tADDRESS port,
  aBool bRead,
  char* data,
  tBYTE dataSize,
  aTEAVMIOPortIOCallback portIO,
  void* ref
)
{
  aErr consoleErr = aErrNone;
  aConsole* pConsole = (aConsole*)ref;
  aPacketRef packet;
#if 0
  unsigned char address;
  unsigned char length;
#endif
  char packetData[aSTEMMAXPACKETBYTES];
  
  aVALIDCONSOLE(pConsole);

  if (bRead) {
    switch (port) {

    case aPortNServo:
    case aPortAddress:
    case aPortNA2D:
    case aPortNDigital:
    case aPortNIR02:
      if (dataSize == 1)
        *data = 0;
      else
        aUtil_StoreShort(data, 0);
      break;

    } /* port switch */

  /* else, write */
  } else {

    switch (port) {

    case aPortNServo:
      consoleErr = aErrIO;
      break;

    case aPortDisplayASCII:
      packetData[0] = (char)cmdVM_MSG;
      packetData[1] = (char)pid;
      packetData[2] = *data;
      if (!aPacket_Create(pConsole->stemLib, 0, /* host = 0 */
      		          3, packetData, &packet, &consoleErr))
        consoleErr = aStemKernel_AddPacket(pConsole->pKernel, packet);
      break;

#if 0
    case aPortCMDOut:
      aAssert(dataSize >= 2);
      address = (unsigned char)data[0];
      length = (unsigned char)data[1];
      if (!aPacket_Create(pConsole->stemLib, address,
      			  length, &data[2], &packet, &consoleErr)) {
      	if (address == 0) {
          consoleErr = aStemKernel_AddPacket(pConsole->pKernel, 
          				     packet);
      	} else {
      	  aStem_SendPacket(pConsole->stemLib, packet, &consoleErr);
      	}
      }
      break;
#endif

    } /* port switch */
  }

  return consoleErr;
  
} /* aConsole_VMPortCB */
