/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaMove.cpp                                         //
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
#include "acpGarciaMove.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the move behaviors

#define aGARCIA_MOVE_PRIMITIVE_NAME	"move"

#define aGPP_MOVE_DISTANCE		"distance"



/////////////////////////////////////////////////////////////////////
// acpGarciaMove constructor
//

acpGarciaMove::acpGarciaMove (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGARCIA_MOVE_PRIMITIVE_NAME, pGarcia)
{
  m_nSlot = 1;
  m_nCodeSize = 888;
  m_nModule = 4;

  m_nDistancePropIndex =
    addProperty(new acpMoveBehaviorDistanceProperty(this));

} // acpGarciaMove constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaMove execute method
//

void acpGarciaMove::execute(acpBehavior* pBehavior)
{
  // get the distance value
  float fDistance = pBehavior->getValue(
  		m_nDistancePropIndex)->getFloatVal();

  char data[aSTEMMAXPACKETBYTES];

  long ticks = (long)(fDistance
		* m_pcGarciaInternal->ticksPerUnitDistance());
  data[0] = cmdVM_RUN;
  data[1] = bitVM_RUN_FIRST 
	    | bitVM_RUN_LAST
    	    | bitVM_RUN_PID;
  data[2] = (char)m_nSlot;
  data[3] = aGARCIA_BEHAVIOR_PID;
  aUtil_StoreLong(&data[4], ticks);
 
  acpPacketMessage message(m_pcGarciaInternal, 
			   m_nModule, 
			   8, 
			   data);

  processMessage(&message);

} // acpGarciaMove execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaMove getParamHTML method
//

const char* acpGarciaMove::getParamHTML()
{
  static char line[500];
  aStringCopy(line, "Distance (");
  aStringCat(line, m_pcGarciaInternal->getNamedValue(
  		   aGARCIA_PROPNAME_DISTUNITSSTR)->getStringVal());
  aStringCat(line, "): <INPUT NAME='distance' VALUE='0' SIZE='5'>");

  return line;

} // acpGarciaMove getParamHTML method



/////////////////////////////////////////////////////////////////////
// distance property
/////////////////////////////////////////////////////////////////////

acpMoveBehaviorDistanceProperty::acpMoveBehaviorDistanceProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_MOVE_DISTANCE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}
