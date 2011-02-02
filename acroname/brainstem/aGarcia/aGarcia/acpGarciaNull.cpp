/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaNull.cpp                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API primitive object. //
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

#include "aUtil.h"
#include "aStem.h"
#include "aGarciaGeom.h"
#include "acpGarciaInternal.h"
#include "acpException.h"
#include "acpHTMLPage.h"
#include "acpBehavior.h"
#include "acpGarciaNull.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the move behaviors

#define aGPP_NULL_PRIMITIVE_NAME	"null"

#define aGPP_NULL_ACCELERATION		"acceleration"



/////////////////////////////////////////////////////////////////////
// acpGarciaNull constructor
//

acpGarciaNull::acpGarciaNull (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGPP_NULL_PRIMITIVE_NAME, pGarcia)
{
  m_nSlot = 5;
  m_nCodeSize = 652;
  m_nModule = 4;

  m_nAccelerationPropIndex = 
    addProperty(new acpNullBehaviorAccelerationProperty(this));

} // acpGarciaNull constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaNull execute method
//

void acpGarciaNull::execute(acpBehavior* pBehavior)
{
  // get the range, side, and mode params
  float fAcc = 
  	pBehavior->getValue(m_nAccelerationPropIndex)->getFloatVal();
  
  char data[aSTEMMAXPACKETBYTES];
  float acctime;

  // (velocity increment per step / acc) * (1000ms per sec)
  acctime = (aGarciaGeom_TicksToSpeed(1, 
   	    m_pcGarciaInternal->distanceUnitType()) / fAcc) * 1000.0f;
  if (acctime < 0.0f) acctime = -acctime;
    
  short sacc = (short)acctime;
  if (sacc < 1) sacc = 1;
  
  data[0] = cmdVM_RUN;
  data[1] = bitVM_RUN_FIRST 
	    | bitVM_RUN_LAST
	    | bitVM_RUN_PID;
  data[2] = (char)m_nSlot;
  data[3] = aGARCIA_BEHAVIOR_PID;
  aUtil_StoreShort(&data[4], sacc);
    
  acpPacketMessage message(m_pcGarciaInternal, m_nModule, 6, data);

  processMessage(&message);
  			      
} // acpGarciaNull execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaNull getParamHTML method
//

const char* acpGarciaNull::getParamHTML()
{
  return "Acceleration: <INPUT NAME='acceleration' VALUE='0' SIZE='5'>";

} // acpGarciaNull getParamHTML method



/////////////////////////////////////////////////////////////////////
// acceleration property
/////////////////////////////////////////////////////////////////////

acpNullBehaviorAccelerationProperty::acpNullBehaviorAccelerationProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_NULL_ACCELERATION,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}
