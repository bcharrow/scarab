/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaPivot.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API pivot primitive.  //
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
#include "acpGarciaPivot.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the pivot behaviors

#define aGARCIA_PIVOT_PRIMITIVE_NAME	"pivot"

#define aGPP_PIVOT_ANGLE		"angle"



/////////////////////////////////////////////////////////////////////
// acpGarciaPivot constructor
//

acpGarciaPivot::acpGarciaPivot (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGARCIA_PIVOT_PRIMITIVE_NAME, pGarcia)
{
  m_nSlot = 2;
  m_nCodeSize = 999;
  m_nModule = 4;

  m_nAnglePropIndex = 
    addProperty(new acpPivotBehaviorAngleProperty(this));

} // acpGarciaPivot constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaPivot execute method
//

void acpGarciaPivot::execute(acpBehavior* pBehavior)
{
  // get the angle value
  float fAngle = pBehavior->getValue(
  			m_nAnglePropIndex)->getFloatVal();

  char data[aSTEMMAXPACKETBYTES];
  
  short ticks = (short)(fAngle 
    			* m_pcGarciaInternal->ticksPerUnitRotation());
  data[0] = cmdVM_RUN;
  data[1] = bitVM_RUN_FIRST 
	    | bitVM_RUN_LAST
    	    | bitVM_RUN_PID;
  data[2] = (char)m_nSlot;
  data[3] = aGARCIA_BEHAVIOR_PID;
  aUtil_StoreShort(&data[4], ticks);
 
  acpPacketMessage message(m_pcGarciaInternal, m_nModule, 6, data);

  processMessage(&message);
  			      
} // acpGarciaPivot execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaPivot getParamHTML method
//

const char* acpGarciaPivot::getParamHTML()
{
  static char line[500];
  aStringCopy(line, "Angle (");
  aStringCat(line, m_pcGarciaInternal->getNamedValue(
  		aGARCIA_PROPNAME_ANGLEUNITSSTR)->getStringVal());
  aStringCat(line, "): <INPUT NAME='angle' VALUE='0' SIZE='5'>");
  
  return line;


} // acpGarciaPivot getParamHTML method



/////////////////////////////////////////////////////////////////////
// angle property
/////////////////////////////////////////////////////////////////////

acpPivotBehaviorAngleProperty::acpPivotBehaviorAngleProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_PIVOT_ANGLE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}
