/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaXMLScript.cpp                                    //
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
#include "acpBehaviorList.h"
#include "acpGarciaXMLScript.h"
#include "acpPacketMessage.h"
#include "acpXMLScript.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the script behavior

#define aGARCIA_SCRIPT_PRIMITIVE_NAME	"script"

#define aGPP_SCRIPT_FILENAME		"filename"



/////////////////////////////////////////////////////////////////////
// acpGarciaXMLScript constructor
//

acpGarciaXMLScript::acpGarciaXMLScript (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGARCIA_SCRIPT_PRIMITIVE_NAME, pGarcia)
{
  m_nSlot = 0;
  m_nCodeSize = 0;
  m_nModule = 0;

  m_nFilenamePropIndex = 
    addProperty(new acpScriptBehaviorFilenameProperty(this));

} // acpGarciaXMLScript constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaXMLScript execute method
//

void acpGarciaXMLScript::execute(acpBehavior* pBehavior)
{
  acpBehaviorList* pList = (acpBehaviorList*)pBehavior;

  // this script parent is giving birth
  // so store number of children
  pList->m_nRefCount = pList->m_childBehaviors.length();

  // go backward through child behaviors and move them
  // to the front of the main behavior queue
  while (!pList->m_childBehaviors.isEmpty()) {
    acpBehavior* pChild;
    pChild = pList->m_childBehaviors.removeTail();
    pList->m_pcGarciaInternal->queueBehaviorHead(pChild);
  }

  // unlink self to force execution of next behavior
  // this script behavior will now dangle
  pBehavior->m_pcGarciaInternal->m_pCurrent = NULL;
   
} // acpGarciaXMLScript execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaXMLScript getParamHTML method
//

const char* acpGarciaXMLScript::getParamHTML()
{
  return "Filename: <INPUT NAME='filename' VALUE='' SIZE='20'>";

} // acpGarciaXMLScript getParamHTML method



/////////////////////////////////////////////////////////////////////
// acpGarciaXMLScript factoryBehavior method
//

acpBehavior* acpGarciaXMLScript::factoryBehavior (
  const char* pBehaviorName,
  acpGarciaPrimitive* pPrimitive,
  const int nID,
  acpGarciaInternal* pGarciaInternal,
  acpBehaviorList* pParent // = NULL
)
{
  return new acpBehaviorList(pBehaviorName, pPrimitive, nID,
    			     pGarciaInternal, pParent); 

} // acpGarciaXMLScript factoryBehavior method



/////////////////////////////////////////////////////////////////////
// filename property
/////////////////////////////////////////////////////////////////////

acpScriptBehaviorFilenameProperty::acpScriptBehaviorFilenameProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_SCRIPT_FILENAME,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_STRING))
{
}
