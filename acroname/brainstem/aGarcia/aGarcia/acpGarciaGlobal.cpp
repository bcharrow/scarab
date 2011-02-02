/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaGlobal.cpp                                       //
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
#include "acpGarciaGlobal.h"
#include "acpPacketMessage.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the move behaviors

#define aGPP_GLOBAL_PRIMITIVE_NAME	"global"



/////////////////////////////////////////////////////////////////////
// acpGarciaGlobal constructor
//

acpGarciaGlobal::acpGarciaGlobal (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaPrimitive(aGPP_GLOBAL_PRIMITIVE_NAME, pGarciaInternal)
{
  m_nSlot = 0;
  m_nCodeSize = 0;
  m_nModule = 0;

} // acpGarciaGlobal constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaGlobal execute method
//

void acpGarciaGlobal::execute(acpBehavior* pBehavior)
{

  // initialize list traversal
  acpListIterator<acpValue> iterVal(pBehavior->m_globalValue);
  acpListIterator<acpInt> iterInt(pBehavior->m_globalIndex);
  acpValue* pValue;
  acpInt* pInt;
  
  // traverse lists and apply Garcia global properties
  while ((pValue = iterVal.next())) {
    pInt = iterInt.next();
    pBehavior->m_pcGarciaInternal->setValue(pInt->m_val, pValue);
  }
  
} // acpGarciaGlobal execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaGlobal getParamHTML method
//

const char* acpGarciaGlobal::getParamHTML()
{
  return "(Not available in this release)";
//"Garcia Property Name: <INPUT NAME='property' VALUE='' SIZE='24'><BR>"
//"Garcia Property Value: <INPUT NAME='value' VALUE='' SIZE='10'><BR>";

} // acpGarciaGlobal getParamHTML method

