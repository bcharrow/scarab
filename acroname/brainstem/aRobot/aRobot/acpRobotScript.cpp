/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotScript.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API script primitive.  //
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
#include "aRobotDefs.tea"

#include "aUtil.h"
#include "aStem.h"

#include "acpRobotInternal.h"
#include "acpException.h"
#include "acpHTMLPage.h"
#include "acpRobotBehaviorList.h"
#include "acpRobotScript.h"
#include "acpRobotPacketMessage.h"
#include "acpRobotXMLReader.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the script behavior

#define aROBOT_SCRIPT_PRIMITIVE_NAME	"script"

#define aROBOT_SCRIPT_FILENAME		"filename"



/////////////////////////////////////////////////////////////////////
// acpRobotScript constructor
//

acpRobotScript::acpRobotScript (
  acpRobotInternal* pRobot
) :
  acpRobotPrimitive(aROBOT_SCRIPT_PRIMITIVE_NAME, pRobot)
{

  m_nFilenamePropIndex = 
    addProperty(new acpScriptBehaviorFilenameProperty(this));

} // acpRobotScript constructor



/////////////////////////////////////////////////////////////////////
// acpRobotScript execute method
//

void acpRobotScript::execute(acpRobotBehavior* pBehavior)
{
  acpRobotBehaviorList* pList = (acpRobotBehaviorList*)pBehavior;

  // if no children then we do nothing
  if (pList->m_childBehaviors.length()) {

    // this script parent is giving birth
    // so store number of children
    pList->m_nRefCount = pList->m_childBehaviors.length();

    // go backward through child behaviors and move them
    // to the front of the main behavior queue
    while (!pList->m_childBehaviors.isEmpty()) {
      acpRobotBehavior* pChild;
      pChild = pList->m_childBehaviors.removeTail();
      pList->m_pcRobotInternal->queueBehaviorHead(pChild);
    }

    // unlink self to force execution of next behavior
    // this script behavior will now dangle
    // until all its children execute
    pBehavior->m_pcRobotInternal->m_pCurrent = NULL;
  } else {
  
    m_pcRobotInternal->finishBehavior(aROBOT_ERRFLAG_NORMAL);
  }
   
} // acpRobotScript execute method



/////////////////////////////////////////////////////////////////////
// acpRobotScript getParamHTML method
//

const char* acpRobotScript::getParamHTML()
{
  return "Filename: <INPUT NAME='filename' VALUE='' SIZE='20'>";

} // acpRobotScript getParamHTML method



/////////////////////////////////////////////////////////////////////
// acpRobotScript factoryBehavior method
//

acpRobotBehavior* acpRobotScript::factoryBehavior (
  const char* pBehaviorName,
  acpRobotPrimitive* pPrimitive,
  const int nID,
  acpRobotInternal* pRobotInternal,
  acpRobotBehaviorList* pParent // = NULL
)
{
  return new acpRobotBehaviorList(pBehaviorName, pPrimitive, nID,
    			     pRobotInternal, pParent); 

} // acpRobotScript factoryBehavior method



/////////////////////////////////////////////////////////////////////
// filename property
/////////////////////////////////////////////////////////////////////

acpScriptBehaviorFilenameProperty::acpScriptBehaviorFilenameProperty (
  				  acpRobotPrimitive* pPrimitive) :
  acpRobotBehaviorProperty(pPrimitive,
  		      aROBOT_SCRIPT_FILENAME,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_STRING))
{
}
