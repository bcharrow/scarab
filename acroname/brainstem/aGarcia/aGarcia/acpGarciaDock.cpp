/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaDock.cpp                                         //
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
#include "acpGarciaDock.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the dock behavior

#define aGARCIA_DOCK_PRIMITIVE_NAME	"dock"

#define aGPP_DOCK_RANGE			"range"



/////////////////////////////////////////////////////////////////////
// acpGarciaDock constructor
//

acpGarciaDock::acpGarciaDock (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive("dock", pGarcia)
{
  m_nSlot = 7;
  m_nCodeSize = 888;
  m_nModule = 4;

  m_nRangePropIndex =
    addProperty(new acpDockBehaviorRangeProperty(this));

} // acpGarciaDock constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaDock execute method
//

void acpGarciaDock::execute(acpBehavior* pBehavior)
{
  // get the distance value
  float fRange = pBehavior->getValue(
  		m_nRangePropIndex)->getFloatVal();
  
  char data[aSTEMMAXPACKETBYTES];

  short srange =
    (short)aGarciaGeom_RangerToRaw(fRange,
				   kGarciaGeomRearRanger,
				   m_pcGarciaInternal->distanceUnitType());
  
  data[0] = cmdVM_RUN;
  data[1] = bitVM_RUN_FIRST 
	    | bitVM_RUN_LAST
	    | bitVM_RUN_PID;
  data[2] = (char)m_nSlot;
  data[3] = aGARCIA_BEHAVIOR_PID;
  aUtil_StoreShort(&data[4], srange);
 
  acpPacketMessage message(m_pcGarciaInternal, m_nModule, 6, data);

  processMessage(&message);

} // acpGarciaDock execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaDock getParamHTML method
//

const char* acpGarciaDock::getParamHTML()
{
  return "Range: <INPUT NAME='range' VALUE='0' SIZE='5'>";

} // acpGarciaDock getParamHTML method



/////////////////////////////////////////////////////////////////////
// distance property
/////////////////////////////////////////////////////////////////////

acpDockBehaviorRangeProperty::acpDockBehaviorRangeProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_DOCK_RANGE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}
