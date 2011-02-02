/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotSleep.cpp                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API sleep primitive.   //
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
#include "acpRobotInternal.h"
#include "acpException.h"
#include "acpHTMLPage.h"
#include "acpRobotBehavior.h"
#include "acpRobotSleep.h"
#include "acpRobotPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the sleep behavior

#define aROBOT_SLEEP_PRIMITIVE_NAME	"sleep"

#define aROBOT_SLEEP_DURATION		"duration"



/////////////////////////////////////////////////////////////////////
// acpRobotSleep constructor
//

acpRobotSleep::acpRobotSleep (
  acpRobotInternal* pRobot
) :
  acpRobotPrimitive("sleep", pRobot)
{
  m_nSlot = 0;
//  m_nCodeSize = 1; // SLEEP IS NOT TREATED AS IMMEDIATE
  m_nModule = 0;

  m_nDurationPropIndex =
    addProperty(new acpSleepDurationProperty(this));

} // acpRobotSleep constructor



/////////////////////////////////////////////////////////////////////
// acpRobotSleep execute method
//

void acpRobotSleep::execute(acpRobotBehavior* pBehavior)
{
  // get the sleep duration
  unsigned long ulTimeMS = (unsigned long)pBehavior->getValue(
  		m_nDurationPropIndex)->getIntVal();
  
  if (ulTimeMS)
    pBehavior->m_pcRobotInternal->goToSleep(ulTimeMS);

} // acpRobotSleep execute method



/////////////////////////////////////////////////////////////////////
// acpRobotSleep getParamHTML method
//

const char* acpRobotSleep::getParamHTML()
{
  return "(Not available in this release)";
//  return "Duration: <INPUT NAME='duration' VALUE='1000' SIZE='5'>";

} // acpRobotSleep getParamHTML method



/////////////////////////////////////////////////////////////////////
// duration property
/////////////////////////////////////////////////////////////////////

acpSleepDurationProperty::acpSleepDurationProperty (
  				  acpRobotPrimitive* pPrimitive) :
  acpRobotBehaviorProperty(pPrimitive,
  		      aROBOT_SLEEP_DURATION,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}
