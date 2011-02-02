/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotGlobal.cpp                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API Global Primitive.  //
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
//#include "aRobotGeom.h"
#include "acpRobotInternal.h"
#include "acpException.h"
//#include "acpHTMLPage.h"
#include "acpRobotBehavior.h"
#include "acpRobotGlobal.h"
#include "acpRobotPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the move behaviors

#define aGPP_GLOBAL_PRIMITIVE_NAME	"global"



/////////////////////////////////////////////////////////////////////
// acpRobotGlobal constructor
//

acpRobotGlobal::acpRobotGlobal (
  acpRobotInternal* pRobotInternal
) :
  acpRobotPrimitive(aGPP_GLOBAL_PRIMITIVE_NAME, pRobotInternal)
{
} // acpRobotGlobal constructor



/////////////////////////////////////////////////////////////////////
// acpRobotGlobal execute method
//

void acpRobotGlobal::execute(acpRobotBehavior* pBehavior)
{

  // initialize list traversal
  acpListIterator<acpValue> iterVal(pBehavior->m_globalValue);
  acpListIterator<acpInt> iterInt(pBehavior->m_globalIndex);
  acpValue* pValue;
  acpInt* pInt;
  
  // traverse lists and apply Robot global properties
  while ((pValue = iterVal.next())) {
    pInt = iterInt.next();
    pBehavior->m_pcRobotInternal->setValue(pInt->m_val, pValue);
  }

    m_pcRobotInternal->finishBehavior(aROBOT_ERRFLAG_NORMAL);
  
} // acpRobotGlobal execute method



/////////////////////////////////////////////////////////////////////
// acpRobotGlobal getParamHTML method
//

const char* acpRobotGlobal::getParamHTML()
{
  return "(Not available in this release)";
//"Robot Property Name: <INPUT NAME='property' VALUE='' SIZE='24'><BR>"
//"Robot Property Value: <INPUT NAME='value' VALUE='' SIZE='10'><BR>";

} // acpRobotGlobal getParamHTML method

