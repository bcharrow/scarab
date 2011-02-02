/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aRLY08.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of cross-platform RLY08 Relay Driver*/
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
#include "aStemMsg.h"
#include "aUtil.h"
#include "aModuleVal.h"
#include "aRLY08.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* local prototypes
*/

static aBool sRLY08IICErrFilter(const unsigned char module,
																const unsigned char dataLength,
																const char* data,
																void* ref);
static aBool sRLY08IICDataFilter(const unsigned char module,
																 const unsigned char dataLength,
																 const char* data,
																 void* ref);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* sRLY08IICErrFilter
*/

aBool sRLY08IICErrFilter(const unsigned char module,
												 const unsigned char dataLength,
												 const char* data,
												 void* ref)
{
  if ((dataLength > 1) 
      && (data[0] == cmdMSG) 
      && (data[1] == iicNoAck))
    return aTrue;
	
  return aFalse;
	
} /* sRLY08IICErrFilter */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sRLY08IICDataFilter
 *
 * This uses the hard-coded device ID of 52 which is valid for 
 * the GP and Moto boards.  It should probably be more general.
 */

aBool sRLY08IICDataFilter(const unsigned char module,
													const unsigned char dataLength,
													const char* data,
													void* ref)
{
  aAssert(data);
	
  if ((dataLength > 1) 
      && (data[0] == cmdDEV_VAL) 
      && (data[1] == 52))
    return aTrue;
	
  return aFalse;
	
} /* sRLY08IICDataFilter */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aRLY08_RelayOn
 *
 * This routine turns a specified relay on.
 */
aErr aRLY08_RelayOn(aStemLib stemLib,
										const unsigned char router,
										const unsigned char rly08address,
										const unsigned char relay)
{
  aErr rly08Err = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
	char iicrate; /* Temp storage for I2C baudrate line */
	
	/* Grab the current I2C baud rate setting */
	aModuleVal_Get(stemLib,router,3,&iicrate);
	
	/* Set the I2C data line to 100kHz */
	aModuleVal_Set(stemLib,router,3,aRLY08_I2CSETTING);
	
  /* check all the parameters and get set up */
  if (!stemLib || !router)
    rly08Err = aErrParam;
		
	/* Check to see if the relay is within range */
	if ((relay < 1) || (relay > 8))
		rly08Err = aErrParam;
	
  /* set the memory pointer in the RLY08 */
  if (rly08Err == aErrNone) {
    data[0] = 0;
    data[1] = (char) (0x64 + relay);
    if (!aPacket_Create(stemLib, 
												rly08address,
												2, 
												data, 
												&packet, &rly08Err))
      aStem_SendPacket(stemLib, packet, &rly08Err);    
  }
	
	/* Set the I2C data line back to what it was before */
	aModuleVal_Set(stemLib,router,3,iicrate);
	
  return rly08Err;
	
} /* aRLY08_RelayOn */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aRLY08_RelayOff
 *
 * This routine turns a specified relay off.
 */
aErr aRLY08_RelayOff(aStemLib stemLib,
										const unsigned char router,
										const unsigned char rly08address,
										const unsigned char relay)
{
  aErr rly08Err = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
	char iicrate; /* Temp storage for I2C baudrate line */
	
	/* Grab the current I2C baud rate setting */
	aModuleVal_Get(stemLib,router,3,&iicrate);
	
	/* Set the I2C data line to 100kHz */
	aModuleVal_Set(stemLib,router,3,aRLY08_I2CSETTING);
	
  /* check all the parameters and get set up */
  if (!stemLib || !router)
    rly08Err = aErrParam;
		
	/* Check to see if the relay is within range */
	if ((relay < 1) || (relay > 8))
		rly08Err = aErrParam;
	
  /* set the memory pointer in the RLY08 */
  if (rly08Err == aErrNone) {
    data[0] = 0;
    data[1] = (char) (0x6E + relay);
    if (!aPacket_Create(stemLib, 
												rly08address,
												2, 
												data, 
												&packet, &rly08Err))
      aStem_SendPacket(stemLib, packet, &rly08Err);    
  }
	
	/* Set the I2C data line back to what it was before */
	aModuleVal_Set(stemLib,router,3,iicrate);
	
  return rly08Err;
	
} /* aRLY08_RelayOff */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aRLY08_RelaysAllOn
 *
 * This routine initiates a turns all the relays on.
 */
aErr aRLY08_RelaysAllOn(aStemLib stemLib,
										const unsigned char router,
										const unsigned char rly08address)
{
  aErr rly08Err = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
	char iicrate; /* Temp storage for I2C baudrate line */
	
	/* Grab the current I2C baud rate setting */
	aModuleVal_Get(stemLib,router,3,&iicrate);
	
	/* Set the I2C data line to 100kHz */
	aModuleVal_Set(stemLib,router,3,aRLY08_I2CSETTING);
	
  /* check all the parameters and get set up */
  if (!stemLib || !router)
    rly08Err = aErrParam;
	
  /* set the memory pointer in the RLY08 */
  if (rly08Err == aErrNone) {
    data[0] = 0;
    data[1] = (char) (0x64);
    if (!aPacket_Create(stemLib, 
												rly08address,
												2, 
												data, 
												&packet, &rly08Err))
      aStem_SendPacket(stemLib, packet, &rly08Err);    
  }
	
	/* Set the I2C data line back to what it was before */
	aModuleVal_Set(stemLib,router,3,iicrate);
	
  return rly08Err;
	
} /* aRLY08_RelaysAllOn */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aRLY08_RelaysAllOff
 *
 * This routine initiates a turns all the relays on.
 */
aErr aRLY08_RelaysAllOff(aStemLib stemLib,
										const unsigned char router,
										const unsigned char rly08address)
{
  aErr rly08Err = aErrNone;
  char data[aSTEMMAXPACKETBYTES];
  aPacketRef packet;
	char iicrate; /* Temp storage for I2C baudrate line */
	
	/* Grab the current I2C baud rate setting */
	aModuleVal_Get(stemLib,router,3,&iicrate);
	
	/* Set the I2C data line to 100kHz */
	aModuleVal_Set(stemLib,router,3,aRLY08_I2CSETTING);
	
  /* check all the parameters and get set up */
  if (!stemLib || !router)
    rly08Err = aErrParam;
	
  /* set the memory pointer in the RLY08 */
  if (rly08Err == aErrNone) {
    data[0] = 0;
    data[1] = (char) (0x6E);
    if (!aPacket_Create(stemLib, 
												rly08address,
												2, 
												data, 
												&packet, &rly08Err))
      aStem_SendPacket(stemLib, packet, &rly08Err);    
  }
	
	/* Set the I2C data line back to what it was before */
	aModuleVal_Set(stemLib,router,3,iicrate);
	
  return rly08Err;
	
} /* aRLY08_RelaysAllOff */
