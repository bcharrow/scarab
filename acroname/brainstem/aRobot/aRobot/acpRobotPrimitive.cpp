/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotPrimitive.cpp                                     //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API primitive object.  //
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

#include "aRobotProperties.h"
#include "acpRobotInternal.h"
#include "acpException.h"
#include "acpHTMLPage.h"
#include "acpRobotBehavior.h"
#include "acpRobotPrimitive.h"
#include "acpRobotProperties.h"
#include "acpRobotPacketMessage.h"



#define aROBOT_BEHAVIOR_PID 0



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitive constructor
//

acpRobotPrimitive::acpRobotPrimitive (
  const char* pName,
  acpRobotInternal* pRobotInternal,
  const bool bHasAPICup, // = false,
  const bool bHasStemCup // = false
) :
  acpObject("primitive", pName),
  m_pcRobotInternal(pRobotInternal),
  m_bHasAPICup(bHasAPICup),
  m_bHasStemCup(bHasStemCup),
  m_nModule(0),
  m_pCode(NULL),
  m_ulCodeSize(0),
  m_nSlot(0),
  m_nUserPropCt(0),
  m_nInputSize(0),
  m_pAPICUP(NULL),
  m_ulAPICUPSize(0)
{
  addProperty(new acpProperty(aROBOT_PROPNAME_UNIQUEID, 
  			      aPROPERTY_FLAG_INT
  			      | aPROPERTY_FLAG_READ));
  acpValue name(pName);
  addProperty(new acpProperty(aROBOT_PROPNAME_PRIMITIVENAME, 
  			      aPROPERTY_FLAG_STRING
  			      | aPROPERTY_FLAG_READ),
  			      &name);
  addProperty(new acpRobotBehaviorExecuteCBProperty(this));
  addProperty(new acpRobotBehaviorCompletionCBProperty(this));
  addProperty(new acpRobotBehaviorStatusProperty(this));
  addProperty(new acpRobotBehaviorExpectedStatusProperty(this));
  addProperty(new acpRobotBehaviorParamHTMLProperty(this));

} // acpRobotPrimitive constructor



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitive constructor
//

acpRobotPrimitive::acpRobotPrimitive (
  acpRobotInternal* pcRobotInternal,
  acpTag_PRIMITIVE* pPrim
) :
  acpObject("primitive", pPrim->m_name),
  m_pcRobotInternal(pcRobotInternal),
  m_bHasAPICup(false),
  m_bHasStemCup(false),
  m_nModule(0),
  m_pCode(NULL),
  m_ulCodeSize(0),
  m_nSlot(0),
  m_nUserPropCt(0),
  m_nInputSize(0),
  m_pAPICUP(NULL),
  m_ulAPICUPSize(0),
  m_nLinkID(aROBOT_APITEA_NOTFOUND)
{
  addProperty(new acpProperty(aROBOT_PROPNAME_UNIQUEID, 
  			      aPROPERTY_FLAG_INT
  			      | aPROPERTY_FLAG_READ));
  acpValue name((const char*)pPrim->m_name);
  addProperty(new acpProperty(aROBOT_PROPNAME_PRIMITIVENAME, 
  			      aPROPERTY_FLAG_STRING
  			      | aPROPERTY_FLAG_READ),
  			      &name);
  addProperty(new acpRobotBehaviorExecuteCBProperty(this));
  addProperty(new acpRobotBehaviorCompletionCBProperty(this));
  addProperty(new acpRobotBehaviorStatusProperty(this));
  addProperty(new acpRobotBehaviorExpectedStatusProperty(this));
  addProperty(new acpRobotBehaviorParamHTMLProperty(this));

  if (pPrim->m_ulApiCupSize)
  {
    m_bHasAPICup = true;

    // copy API TEA executable
    m_ulAPICUPSize = pPrim->m_ulApiCupSize;
    m_pAPICUP = (char*)aMemAlloc(m_ulAPICUPSize * sizeof(char));
    aMemCopy(m_pAPICUP, pPrim->m_pApiCup, m_ulAPICUPSize);

    if (pPrim->m_ulStemCupSize) {

      m_bHasStemCup = true;

      // add Stem CUP properties
      acpValue v(-1);
      addProperty(new acpProperty(aROBOT_PROPNAME_MODULE, 
				  aPROPERTY_FLAG_INT
				  | aPROPERTY_FLAG_WRITE
				  | aPROPERTY_FLAG_READ),
				  &v);
      addProperty(new acpProperty(aROBOT_PROPNAME_FILESLOT, 
				  aPROPERTY_FLAG_INT
				  | aPROPERTY_FLAG_WRITE
				  | aPROPERTY_FLAG_READ),
				  &v);
      addProperty(new acpProperty(aROBOT_PROPNAME_PROCESSID, 
				  aPROPERTY_FLAG_INT
				  | aPROPERTY_FLAG_WRITE
				  | aPROPERTY_FLAG_READ),
				  &v);

      // copy Stem TEA executable
      m_ulCodeSize = pPrim->m_ulStemCupSize;
      m_pCode = (char*)aMemAlloc(m_ulCodeSize * sizeof(char));
      aMemCopy(m_pCode, pPrim->m_pStemCup, m_ulCodeSize);
    }
  }

} // acpRobotPrimitive constructor



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitive destructor
//

acpRobotPrimitive::~acpRobotPrimitive() 
{
  if (m_pAPICUP)
    aMemFree(m_pAPICUP);
  if (m_pCode)
    aMemFree(m_pCode);

} // acpRobotPrimitive destructor



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitive addProperty method
//

int acpRobotPrimitive::addPrimitiveProperty(
  acpRobotUserProperty* pProperty,
  acpValue* pDefault
)
{
  // primitive requires a default and
  // some extra bookkeeping is handy
  m_nUserPropCt++;
  m_nInputSize += pProperty->m_tSetBlock.cParamSize;

  return acpObject::addProperty(pProperty, pDefault); 

} // addProperty method



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitive execute method
//

void acpRobotPrimitive::execute(acpRobotBehavior* pBehavior)
{
  // allocate input buffer
  char* pInputBuff = (char*)aMemAlloc(m_nInputSize * sizeof(char));

  // populate input buffer
  // walk backwards through user properties
  // (there is a fixed number of them at end of list)
  int numProp = numProperties() - 1;
  int k;
  int i;
  for (k = 0, i = numProp; k < m_nUserPropCt; k++, i--) {
    acpRobotUserProperty* pProp = 
      (acpRobotUserProperty*) getProperty(i);
#ifdef aDEBUG
    aPROPERTY_FLAGS flags = pProp->getTypeFlags();
    aAssert(flags & aPROPERTY_FLAG_USERBIT);
#endif // aDEBUG
    pProp->convertValToInput(pBehavior->getValue(i), pInputBuff);
  }

  // the robot thread makes the call to execute primitives
  if (m_pAPICUP) {

    char cStemModule = 0;
    char cStemProcID = 0;

    if (m_bHasStemCup) {
      cStemModule = (char)
        getNamedValue(aROBOT_PROPNAME_MODULE)->getIntVal();
      cStemProcID = (char)
        getNamedValue(aROBOT_PROPNAME_PROCESSID)->getIntVal();
    }

    m_pcRobotInternal->m_pcVMM[aROBOT_APITEA_PRIMPROCID]->
      withinThreadLaunch(
        m_nLinkID,
        m_pAPICUP,
	m_ulAPICUPSize,
	pInputBuff,
	acpRobotPrimitive::m_vmExitProc,
	this);
  }

  // thread launch copies input data
  // so we can delete this copy
  aMemFree(pInputBuff);

} // acpRobotPrimitive execute method



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitive getParamHTML method
//

const char* acpRobotPrimitive::getParamHTML()
{
  // ???
  static char line[500];
  aStringCopy(line, "foo");
  return line;

} // acpGarciaMove getParamHTML method



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitives factoryBehavior method
//

acpRobotBehavior* acpRobotPrimitive::factoryBehavior (
  const char* pBehaviorName,
  acpRobotPrimitive* pPrimitive,
  const int nID,
  acpRobotInternal* pRobotInternal,
  acpRobotBehaviorList* pParent // = NULL
)
{
  return new acpRobotBehavior(pBehaviorName, 
  			 pPrimitive, nID,
    			 pRobotInternal, 
    			 pParent); 

} // acpRobotPrimitive factoryBehavior method



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitive factoryBehavior method
//

acpRobotBehavior* acpRobotPrimitive::factoryBehavior (
  acpHTMLPage& page
)
{
  // build the behavior first
  acpRobotBehavior* pBehavior = 
  	m_pcRobotInternal->createNamedBehavior(
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

} // acpRobotPrimitive factoryBehavior method



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitives processMessage method
//

void acpRobotPrimitive::processMessage(acpRobotPacketMessage* pPacketMessage)
{ 
  m_pcRobotInternal->handlePacketOut(pPacketMessage);

} // acpRobotPrimitives processMessage method



/////////////////////////////////////////////////////////////////////
// acpRobotPrimitives processMessage method
//

const char* acpRobotPrimitive::getBasicPrimitiveHTML()
{
  return "Name: <INPUT NAME='name' VALUE='' SIZE='10'>";
}


/////////////////////////////////////////////////////////////////////
// exit callback specific to primitive
//

aErr acpRobotPrimitive::m_vmExitProc(
  const aVMExit eExitCode,
  const char* returnData,
  const unsigned char returnDataSize,
  const aTEAProcessID pid,
  const void* ref
)
{
  short status = aROBOT_ERRFLAG_NORMAL;
  aAssert(pid == aROBOT_APITEA_PRIMPROCID);

  // retrieve identity
  acpRobotPrimitive* pPrim =
    (acpRobotPrimitive*)ref;

  // get machinery
  acpRobotInternal* pBot =
    pPrim->m_pcRobotInternal;
  acpRobotVMManager* pcVMM =
    pBot->m_pcVMM[aROBOT_APITEA_PRIMPROCID];

  if ((eExitCode != aVMExitNormal) &&
      (eExitCode != aVMExitKill)) {

    // something bad happened while executing an API VM opcode
    char msg[aFILE_NAMEMAXCHARS];
    aStringCopy(msg, "Primitive API TEA error: ");
    aStringCat(msg, pPrim->getNamedValue("name")->getStringVal());
    pBot->logMessage(pPrim->m_nLinkID, (int)eExitCode, msg);

    // we have an API-level error
    status = aROBOT_ERRFLAG_APIVMERROR;

  } else {

    if (eExitCode == aVMExitKill) {
      // we aborted the API TEA program
      status = aROBOT_ERRFLAG_ABORT;
    } else {
      char cLen = pcVMM->getRxLength();
      if (cLen > 4) {
        // normal exit from Stem Program
        status = aUtil_RetrieveShort(pcVMM->getRxDataPtr(4));
      } else if (cLen == 0) {
        // normal exit from API Program
        status = aUtil_RetrieveShort(pcVMM->getRxDataPtr(4));
      } else {
        // abnormal exit from Stem Program
        // may still need work here ???
        status = aROBOT_ERRFLAG_ABORT;
      }
    }
  }

  // if here then we are done
  // (we are also in the thread)
  pBot->finishBehavior(status);
  pcVMM->setNotActive();
  return aErrNone;
}
