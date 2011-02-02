/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaHug.cpp                                         //
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
#include "acpGarciaHug.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the align behaviors

#define aGARCIA_HUG_PRIMITIVE_NAME	"hug"

#define aGPP_HUG_RANGE			"range"
#define aGPP_HUG_SIDE			"side"
#define aGPP_HUG_MODE			"mode"



/////////////////////////////////////////////////////////////////////
// acpGarciaHug constructor
//

acpGarciaHug::acpGarciaHug (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGARCIA_HUG_PRIMITIVE_NAME, pGarcia)
{
  m_nSlot = 3;
  m_nCodeSize = 918;
  m_nModule = 4;

  m_nRangePropIndex = 
    addProperty(new acpHugBehaviorRangeProperty(this));
  m_nSidePropIndex = 
    addProperty(new acpHugBehaviorSideProperty(this));
  m_nModePropIndex = 
    addProperty(new acpHugBehaviorModeProperty(this));

} // acpGarciaHug constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaHug execute method
//

void acpGarciaHug::execute(acpBehavior* pBehavior)
{
  // get the params
  char cSide = (char)pBehavior->getValue(
  			m_nSidePropIndex)->getIntVal();
  char cMode = (char)pBehavior->getValue(
  			m_nModePropIndex)->getIntVal();
  float fRange = pBehavior->getValue(
  			m_nRangePropIndex)->getFloatVal();
  
  char data[aSTEMMAXPACKETBYTES];

  short srange =
      (short)aGarciaGeom_RangerToRaw(fRange,
				     kGarciaGeomSideRanger,
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
  			      
} // acpGarciaHug execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaHug getParamHTML method
//

const char* acpGarciaHug::getParamHTML()
{
  static char line[500];
  aStringCopy(line, "Side: <SELECT NAME='side'>");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "3");
  aStringCat(line, "'>Right");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "2");
  aStringCat(line, "'>Left");
  aStringCat(line, "</SELECT><BR>");
  aStringCat(line, "Mode: <SELECT NAME='mode'>");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "0");
  aStringCat(line, "'>Default");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "1");
  aStringCat(line, "'>Side loss");
  aStringCat(line, "<OPTION VALUE='");
  aStringCat(line, "2");
  aStringCat(line, "'>Side bump");
  aStringCat(line, "</SELECT><BR>Range (");
  aStringCat(line, m_pcGarciaInternal->getNamedValue(
  			aGARCIA_PROPNAME_DISTUNITSSTR)->getStringVal());
  aStringCat(line, "): <INPUT NAME='range' VALUE='0' SIZE='5'>");
  
  return line;


} // acpGarciaHug getParamHTML method



/////////////////////////////////////////////////////////////////////
// range property
/////////////////////////////////////////////////////////////////////

acpHugBehaviorRangeProperty::acpHugBehaviorRangeProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_HUG_RANGE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
// side property
/////////////////////////////////////////////////////////////////////

acpHugBehaviorSideProperty::acpHugBehaviorSideProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_HUG_SIDE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}



/////////////////////////////////////////////////////////////////////
// mode property
/////////////////////////////////////////////////////////////////////

acpHugBehaviorModeProperty::acpHugBehaviorModeProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_HUG_MODE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_INT))
{
}
