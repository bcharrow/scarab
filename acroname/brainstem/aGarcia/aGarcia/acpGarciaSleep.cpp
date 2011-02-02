/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaSleep.cpp                                        //
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
#include "acpGarciaSleep.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the sleep behavior

#define aGARCIA_SLEEP_PRIMITIVE_NAME	"sleep"

#define aGPP_SLEEP_DURATION		"duration"



/////////////////////////////////////////////////////////////////////
// acpGarciaSleep constructor
//

acpGarciaSleep::acpGarciaSleep (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive("sleep", pGarcia)
{
  m_nSlot = 0;
  m_nCodeSize = 1; // SLEEP IS NOT TREATED AS IMMEDIATE
  m_nModule = 0;

  m_nDurationPropIndex =
    addProperty(new acpSleepBehaviorDurationProperty(this));

} // acpGarciaSleep constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaSleep execute method
//

void acpGarciaSleep::execute(acpBehavior* pBehavior)
{
  // get the sleep duration
  unsigned long ulTimeMS = (unsigned long)pBehavior->getValue(
  		m_nDurationPropIndex)->getIntVal();
  
  if (ulTimeMS)
    pBehavior->m_pcGarciaInternal->goToSleep(ulTimeMS);

} // acpGarciaSleep execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaSleep getParamHTML method
//

const char* acpGarciaSleep::getParamHTML()
{
  return "Duration: <INPUT NAME='duration' VALUE='1000' SIZE='5'>";

} // acpGarciaSleep getParamHTML method



/////////////////////////////////////////////////////////////////////
// duration property
/////////////////////////////////////////////////////////////////////

acpSleepBehaviorDurationProperty::acpSleepBehaviorDurationProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_SLEEP_DURATION,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}
