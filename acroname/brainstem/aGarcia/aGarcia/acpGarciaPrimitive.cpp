/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaPrimitive.cpp                                    //
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
#include "acpGarciaPrimitive.h"
#include "acpPacketMessage.h"



#define aGARCIA_BEHAVIOR_PID 0



/////////////////////////////////////////////////////////////////////
// acpGarciaPrimitive constructor
//

acpGarciaPrimitive::acpGarciaPrimitive (
  const char* pName,
  acpGarciaInternal* pGarciaInternal
) :
  acpObject("primitive", pName),
  m_pcGarciaInternal(pGarciaInternal),
  m_nModule(4),
  m_pCode(NULL),
  m_nCodeSize(777),
  m_nSlot(1)
{
  addProperty(new acpProperty(aGARCIA_PROPNAME_UNIQUEID, 
  			      aPROPERTY_FLAG_INT
  			      | aPROPERTY_FLAG_READ));
  acpValue name(pName);
  addProperty(new acpProperty(aGARCIA_PRIMITIVE_NAME, 
  			      aPROPERTY_FLAG_STRING
  			      | aPROPERTY_FLAG_READ),
  			      &name);
  addProperty(new acpBehaviorExecuteCBProperty(this));
  addProperty(new acpBehaviorCompletionCBProperty(this));
  addProperty(new acpBehaviorStatusProperty(this));
  addProperty(new acpBehaviorExpectedStatusProperty(this));

} // acpGarciaPrimitive constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaPrimitive destructor
//

acpGarciaPrimitive::~acpGarciaPrimitive() 
{
} // acpGarciaPrimitive destructor



/////////////////////////////////////////////////////////////////////
// acpGarciaPrimitives factoryBehavior method
//

acpBehavior* acpGarciaPrimitive::factoryBehavior (
  const char* pBehaviorName,
  acpGarciaPrimitive* pPrimitive,
  const int nID,
  acpGarciaInternal* pGarciaInternal,
  acpBehaviorList* pParent // = NULL
)
{
  return new acpBehavior(pBehaviorName, 
  			 pPrimitive, nID,
    			 pGarciaInternal, 
    			 pParent); 

} // acpGarciaPrimitive factoryBehavior method



/////////////////////////////////////////////////////////////////////
// acpGarciaPrimitive factoryBehavior method
//

acpBehavior* acpGarciaPrimitive::factoryBehavior (
  acpHTMLPage& page
)
{
  // build the behavior first
  acpBehavior* pBehavior = 
  	m_pcGarciaInternal->createNamedBehavior(
  		getValue(m_nNamePropertyIndex)->getStringVal(), 
  				   page.getStringParam("name"));

  // walk the primitive properties and set the writable ones
  // (the properties that come after the base properties)
  // on the new behavior
  int q2 = pBehavior->numProperties();
  int i;
  
  for (i = 0; i < q2; i++) {
    char* pPropName = pBehavior->getPropertyName(i);
    aPROPERTY_FLAGS flags = pBehavior->getPropertyFlags(i);
    if (flags & aPROPERTY_FLAG_USERBIT) {

      acpValue val;

      if (flags & aPROPERTY_FLAG_FLOAT)
        val.set(page.getFloatParam(pPropName));
      else if (flags & aPROPERTY_FLAG_INT)
        val.set(page.getIntParam(pPropName));
      else if (flags & aPROPERTY_FLAG_STRING)
        val.set(page.getStringParam(pPropName));
      
      pBehavior->setValue(i, &val);
    }  
  }

  return pBehavior;

} // acpGarciaPrimitive factoryBehavior method



/////////////////////////////////////////////////////////////////////
// acpGarciaPrimitives processMessage method
//

void acpGarciaPrimitive::processMessage(acpPacketMessage* pPacketMessage)
{ 
  m_pcGarciaInternal->handlePacketOut(pPacketMessage);

} // acpGarciaPrimitives processMessage method



/////////////////////////////////////////////////////////////////////
// acpGarciaPrimitives processMessage method
//

const char* acpGarciaPrimitive::getBasicPrimitiveHTML()
{
  return "Name: <INPUT NAME='name' VALUE='' SIZE='10'>";
}

