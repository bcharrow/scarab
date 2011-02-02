/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaTurn.cpp                                         //
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
#include "acpGarciaInternal.h"
#include "acpException.h"
#include "acpHTMLPage.h"
#include "acpBehavior.h"
#include "acpGarciaTurn.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the turn behavior

#define aGARCIA_TURN_PRIMITIVE_NAME	"turn"

#define aGPP_TURN_ANGLE			"angle"
#define aGPP_TURN_SIDE			"side"



/////////////////////////////////////////////////////////////////////
// acpGarciaTurn constructor
//

acpGarciaTurn::acpGarciaTurn (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGARCIA_TURN_PRIMITIVE_NAME, pGarcia)
{
  m_nSlot = 0;
  m_nCodeSize = 934;
  m_nModule = 4;

  m_nAnglePropIndex = 
    addProperty(new acpTurnBehaviorAngleProperty(this));
  m_nSidePropIndex = 
    addProperty(new acpTurnBehaviorSideProperty(this));

} // acpGarciaTurn constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaTurn execute method
//

void acpGarciaTurn::execute(acpBehavior* pBehavior)
{
  // get the angle and side params
  char cSide = (char)pBehavior->getValue(
  			m_nSidePropIndex)->getIntVal();
  float fAngle = pBehavior->getValue(
  			m_nAnglePropIndex)->getFloatVal();

  char data[aSTEMMAXPACKETBYTES];

  short ticks = (short)(fAngle 
		* 2.0 * m_pcGarciaInternal->ticksPerUnitRotation());
  
  data[0] = cmdVM_RUN;
  data[1] = bitVM_RUN_FIRST 
	    | bitVM_RUN_LAST
	    | bitVM_RUN_PID;
  data[2] = (char)m_nSlot;
  data[3] = aGARCIA_BEHAVIOR_PID;
  data[4] = cSide;
  aUtil_StoreShort(&data[5], ticks);
    
  acpPacketMessage message(m_pcGarciaInternal, m_nModule, 7, data);

  processMessage(&message);
  			      
} // acpGarciaTurn execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaTurn getParamHTML method
//

const char* acpGarciaTurn::getParamHTML()
{
  static char line[500];
  aStringCopy(line, "Drive side: <SELECT NAME='side'>");
  aStringCat(line, "<OPTION VALUE='0'>Right (0)");
  aStringCat(line, "<OPTION VALUE='1'>Left (1)");
  aStringCat(line, "</SELECT><BR>Angle (");
  aStringCat(line, m_pcGarciaInternal->getNamedValue(
  			aGARCIA_PROPNAME_ANGLEUNITSSTR)->getStringVal());
  aStringCat(line, "): <INPUT NAME='angle' VALUE='0' SIZE='5'>");
  
  return line;

} // acpGarciaTurn getParamHTML method



/////////////////////////////////////////////////////////////////////
// angle property
/////////////////////////////////////////////////////////////////////

acpTurnBehaviorAngleProperty::acpTurnBehaviorAngleProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_TURN_ANGLE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
// side property
/////////////////////////////////////////////////////////////////////

acpTurnBehaviorSideProperty::acpTurnBehaviorSideProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_TURN_SIDE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}
