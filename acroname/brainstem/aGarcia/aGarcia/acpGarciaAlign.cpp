/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaAlign.cpp                                         //
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
#include "acpGarciaAlign.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the align behaviors

#define aGARCIA_ALIGN_PRIMITIVE_NAME	"align"

#define aGPP_ALIGN_RANGE		"range"
#define aGPP_ALIGN_SIDE			"side"
#define aGPP_ALIGN_MODE			"mode"



/////////////////////////////////////////////////////////////////////
// acpGarciaAlign constructor
//

acpGarciaAlign::acpGarciaAlign (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGARCIA_ALIGN_PRIMITIVE_NAME, pGarcia)
{
  m_nSlot = 4;
  m_nCodeSize = 846;
  m_nModule = 4;

  m_nRangePropIndex = 
    addProperty(new acpAlignBehaviorRangeProperty(this));
  m_nSidePropIndex = 
    addProperty(new acpAlignBehaviorSideProperty(this));
  m_nModePropIndex = 
    addProperty(new acpAlignBehaviorModeProperty(this));

} // acpGarciaAlign constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaAlign execute method
//

void acpGarciaAlign::execute(acpBehavior* pBehavior)
{
  // get the params
  char cSide = (char)pBehavior->getValue(
  			m_nSidePropIndex)->getIntVal();
  char cMode = (char)pBehavior->getValue(
  			m_nModePropIndex)->getIntVal();
  float fRange = pBehavior->getValue(
  			m_nRangePropIndex)->getFloatVal();
  
  if (1) {
    char data[aSTEMMAXPACKETBYTES];

    short srange =
      (short)aGarciaGeom_RangerToRaw(fRange,
				     kGarciaGeomFrontRanger,
				     m_pcGarciaInternal->distanceUnitType());
  
    data[0] = cmdVM_RUN;
    data[1] = bitVM_RUN_FIRST 
    	      | bitVM_RUN_LAST
    	      | bitVM_RUN_PID;
    data[2] = (char)m_nSlot;
    data[3] = aGARCIA_BEHAVIOR_PID;
    data[4] = cSide;
    data[5] = cMode;
    aUtil_StoreShort(&data[6], srange);
    
    acpPacketMessage message(m_pcGarciaInternal, m_nModule, 8, data);

    processMessage(&message);
  }

} // acpGarciaAlign execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaAlign getParamHTML method
//

const char* acpGarciaAlign::getParamHTML()
{
  static char line[500];
  aStringCopy(line, "Side: <SELECT NAME='side'>");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "1");
  aStringCat(line, "'>Right Sensor");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "0");
  aStringCat(line, "'>Left Sensor");
  aStringCat(line, "</SELECT><BR>");
  aStringCat(line, "Mode: <SELECT NAME='mode'>");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "0");
  aStringCat(line, "'>Default");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "1");
  aStringCat(line, "'>Rotate");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "2");
  aStringCat(line, "'>Straight");
  aStringCat(line, "</SELECT><BR>Range (");
  aStringCat(line, m_pcGarciaInternal->getNamedValue(
  			aGARCIA_PROPNAME_DISTUNITSSTR)->getStringVal());
  aStringCat(line, "): <INPUT NAME='range' VALUE='0' SIZE='5'>");
  
  return line;

} // acpGarciaAlign getParamHTML method



/////////////////////////////////////////////////////////////////////
// range property
/////////////////////////////////////////////////////////////////////

acpAlignBehaviorRangeProperty::acpAlignBehaviorRangeProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_ALIGN_RANGE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
// side property
/////////////////////////////////////////////////////////////////////

acpAlignBehaviorSideProperty::acpAlignBehaviorSideProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_ALIGN_SIDE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}



/////////////////////////////////////////////////////////////////////
// mode property
/////////////////////////////////////////////////////////////////////

acpAlignBehaviorModeProperty::acpAlignBehaviorModeProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_ALIGN_MODE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}
