/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetRangerValueMessage.cpp                              //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the GetRangerValueMessage object.//
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

#include "aCmd.tea"
#include "aGarciaDefs.tea"
#include "acpGetRangerValueMessage.h"
#include "aGarciaGeom.h"
#include "aStemCore.h"
#include "aUtil.h"

#define aRANGER_TIMEOUT		400



/////////////////////////////////////////////////////////////////////
// acpGetRangerValueMessage process method
//

void acpGetRangerValueMessage::process()
{
  aErr err;
  unsigned char ucaddress;
  unsigned char uclength;
  unsigned short unRawValue = 0;

  aPacketRef packet;
  char xbytes[aSTEMMAXPACKETBYTES];

  eGarciaRangerType eRangerType = kGarciaGeomNoRanger;

  aAssert(m_pcGarcia->m_pcThread->isThreadContext());
  

  // initialize bookkeeping stuff for the desired ranger
  switch(m_cID) {
    case aGARCIA_GP_BIT_ENABLESIDE:
      eRangerType = kGarciaGeomSideRanger;
      break;
    case aGARCIA_GP_BIT_ENABLEFRONT:
      eRangerType = kGarciaGeomFrontRanger;
      break;
    case aGARCIA_GP_BIT_ENABLEREAR:
      eRangerType = kGarciaGeomRearRanger;
      break;
    case aGARCIA_GP_BIT_ENABLEDOWN:
      eRangerType = kGarciaGeomDownRanger;
      break;
  }

  // trigger reflex to read appropriate sensor
  // and turn on sensor enablers if necessary
  // (reflexe enabler check command propagates reflex)
  xbytes[0] = cmdRFLXE_CHK;
  xbytes[1] = m_cID;
  xbytes[2] = m_cCode;
  aPacket_Create(m_pcGarcia->m_stemRef,
		 aGARCIA_GP_ADDR,
		 3,
		 xbytes,
		 &packet, &err);
  aAssert(err == aErrNone);
  aStem_SendPacket(m_pcGarcia->m_stemRef, packet, &err);

  // seek the cmdDEV_VAL reply
  if (err == aErrNone) {
    if (!aStem_GetPacket(m_pcGarcia->m_stemRef, 
    			 aStemCore_CmdFilter, 
			 (void*)cmdDEV_VAL,
      		         aRANGER_TIMEOUT, 
      		         &packet, 
      		         &err)
        && !aPacket_GetData(m_pcGarcia->m_stemRef, 
        		    packet, 
        		    &ucaddress,
      		            &uclength, 
      		            xbytes, 
      		            &err)) {
      if (uclength == 3) {
        unsigned char uval = (unsigned char)xbytes[2];
        unRawValue = uval;
      } else if (uclength == 4) {
        unRawValue = aUtil_RetrieveShort(&xbytes[2]);
        unRawValue >>= 6;
      } else {
        err = aErrNotFound;
      }
      aPacket_Destroy(m_pcGarcia->m_stemRef, packet, NULL);
    }
  }

  // raw value is unsigned short to prevent sign flip problems
  if (err == aErrNone) {

    if (eRangerType == kGarciaGeomDownRanger) {

      m_pValue->set((int)unRawValue);

    } else {

      float fValue;
      fValue =
        aGarciaGeom_ScaleRangerData(
          unRawValue,
          eRangerType,
          m_pcGarcia->m_nDistanceUnitType);
      m_pValue->set((float)fValue);
    }
  }
  
} // acpGetRangerValueMessage::process method
