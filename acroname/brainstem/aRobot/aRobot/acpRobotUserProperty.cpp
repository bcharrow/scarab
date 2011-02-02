/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotUserProperty.cpp                                  //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API                    //
//              user defined property.                             //
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
#include "aStemCore.h"

#include "acpTag_PROPERTY.h"
#include "acpTag_SET.h"
#include "acpTag_GET.h"

#include "aRobotDefs.tea"
#include "aRobotProperties.h"
#include "acpRobotProperties.h"
#include "acpRobotVMLaunchMessage.h"



/////////////////////////////////////////////////////////////////////
// acpRobotTEAProperty::acpRobotTEAProperty

acpRobotTEAProperty::acpRobotTEAProperty(
  acpRobotInternal* pRobotInternal,
  const char* pName,
  aPROPERTY_FLAGS flags,
  const char* pDescription,
  const tConvBlock* pSetBlock,
  const tConvBlock* pGetBlock) :
  acpRobotProperty(pRobotInternal, pName, flags, pDescription),
  m_nLinkID(aROBOT_APITEA_NOTFOUND)
{
  if (pSetBlock)
    aMemCopy(&m_tSetBlock, pSetBlock, sizeof(tConvBlock));
  if (pGetBlock)
    aMemCopy(&m_tGetBlock, pGetBlock, sizeof(tConvBlock));
}


/////////////////////////////////////////////////////////////////////
// acpRobotTEAProperty::convertValToInput

void acpRobotTEAProperty::convertValToInput(
  const acpValue* pValue,
  char* pInputBuff
)
{  
  aPROPERTY_FLAGS flags = getTypeFlags();

  long lParam;
  char* pDest = &pInputBuff[m_tSetBlock.cParamIndex];

  // convert from prop type to long
  // others ???
  lParam = 0;
  if (flags & aPROPERTY_FLAG_FLOAT) {
    float f = pValue->getFloatVal();
    f = m_pRobotInternal->
          convertFromGlobalUnits(f, m_cUnitType, m_cUnitCode);
    lParam = (long)(f * m_tSetBlock.ulUnit);
  } else if (flags & aPROPERTY_FLAG_INT) {
    int n = pValue->getIntVal();
// is there any point in doing unit conversion on int types???
//    n = (int)m_pcRobotInternal->
//          convertToGlobalUnits((float)n, m_cUnitType, m_cUnitCode);
    lParam = (long)(n * m_tSetBlock.ulUnit);
  } else if (flags & aPROPERTY_FLAG_BOOL) {
    lParam = (long)pValue->getBoolVal();
  }

  // convert to an int type of appropriate
  // size and stuff into temp buffer
  switch(m_tSetBlock.cParamSize) {
    case 1:
      pDest[0] = (char)lParam;
      break;
    case 2:
      aUtil_StoreShort(pDest, (short)lParam);
      break;
    case 4:
      aUtil_StoreLong(pDest, lParam);
      break;
  }
}


/////////////////////////////////////////////////////////////////////
// acpRobotTEAProperty::convertOutputToVal

void acpRobotTEAProperty::convertOutputToVal(
  acpValue* pValue,
  const char* pOutputBuff
)
{

  aPROPERTY_FLAGS flags = getTypeFlags();
  long lParam;

  // convert from variable sized int
  // to a fixed-size long
  lParam = 0;
  switch(m_tGetBlock.cParamSize) {
    case 1:
      lParam = (long)pOutputBuff[0];
      break;
    case 2:
      lParam = (long)aUtil_RetrieveShort(pOutputBuff);
      break;
    case 4:
      lParam =
        (long)(((pOutputBuff[0] << 24) & 0xFF000000) |
               ((pOutputBuff[1] << 16) & 0x00FF0000) |
               ((pOutputBuff[2] <<  8) & 0x0000FF00) |
                (pOutputBuff[3]        & 0x000000FF));
      break;
  }

  // convert from long to value type
  if (flags & aPROPERTY_FLAG_FLOAT) {
    float f = ((float)lParam / (float)m_tGetBlock.ulUnit);
    f = m_pRobotInternal->
          convertToGlobalUnits(f, m_cUnitType, m_cUnitCode);
    pValue->set(f);
  } else if (flags & aPROPERTY_FLAG_INT) {
    int n = (int)lParam / m_tGetBlock.ulUnit;
// is there any point in doing unit conversion on int types???
//    n = (int)m_pcRobotInternal->
//          convertToGlobalUnits((float)n, m_cUnitType, m_cUnitCode);
    pValue->set(n);
  } else if (flags & aPROPERTY_FLAG_BOOL) {
    bool b = (bool)lParam;
    pValue->set(b);
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotTEAProperty::addTagData(
  acpPackageTag* pTag
)
{
  switch (pTag->getTagType()) {
    case aTAG_SET:
    {
      acpTag_SET* pSet = (acpTag_SET*)pTag;
      m_tSetBlock.ulUnit = pSet->m_nUnit;
      m_tSetBlock.cParamIndex = pSet->m_cParamIndex;
      m_tSetBlock.cParamSize = pSet->m_cParamSize;
      m_tSetBlock.nConvType = 0;
      break;
    }
    case aTAG_GET:
    {
      acpTag_GET* pGet = (acpTag_GET*)pTag;
      m_tGetBlock.ulUnit = pGet->m_nUnit;
      m_tGetBlock.cParamIndex = pGet->m_cParamIndex;
      m_tGetBlock.cParamSize = pGet->m_cParamSize;
      m_tGetBlock.nConvType = 0;
      break;
    }
  }
}


/////////////////////////////////////////////////////////////////////

acpRobotUserProperty::acpRobotUserProperty (
  acpRobotInternal* pcRobotInternal,
  acpTag_PROPERTY* pProp
) :
  acpRobotTEAProperty(
    pcRobotInternal,
    pProp->m_name,
    pProp->m_flags,
    pProp->m_description,
    NULL,
    NULL),
  m_pSetCUP(NULL),
  m_pGetCUP(NULL)
{
  m_cUnitType = pProp->m_unittype;
  m_cUnitCode = pProp->m_unitcode;
}


/////////////////////////////////////////////////////////////////////

acpRobotUserProperty::~acpRobotUserProperty ()
{
  if (m_pSetCUP)
    aMemFree(m_pSetCUP);
  if (m_pGetCUP)
    aMemFree(m_pGetCUP);
}


/////////////////////////////////////////////////////////////////////
// allocate CUP files via tag data
//

void acpRobotUserProperty::addTagData(
  acpPackageTag* pTag
)
{
  switch (pTag->getTagType()) {
    case aTAG_SET:
    {
      acpTag_SET* pSet = (acpTag_SET*)pTag;
      m_ulSetSize = pSet->m_ulApiCupSize;
      if (m_ulSetSize)
        m_pSetCUP = (char*)aMemAlloc(m_ulSetSize * sizeof(char));
      if (m_pSetCUP)
        aMemCopy(m_pSetCUP, pSet->m_pApiCup, m_ulSetSize);
      break;
    }
    case aTAG_GET:
    {
      acpTag_GET* pGet = (acpTag_GET*)pTag;
      m_ulGetSize = pGet->m_ulApiCupSize;
      if (m_ulGetSize)
        m_pGetCUP = (char*)aMemAlloc(m_ulGetSize * sizeof(char));
      if (m_pGetCUP)
        aMemCopy(m_pGetCUP, pGet->m_pApiCup, m_ulGetSize);
      break;
    }
  }
  acpRobotTEAProperty::addTagData(pTag);
}


/////////////////////////////////////////////////////////////////////
// set implemented with TEA script
//

void acpRobotUserProperty::setValue(
  const acpValue* pValue
)
{
  // others ???
  // parameters have at most 4 bytes
  char cInputData[4];

  // set up state for when we enter static exit callback
  m_bSet = true;
  m_pValue = NULL;
  m_bDone = false;

  convertValToInput(pValue, cInputData);

  if (m_pSetCUP) {

    if (m_pRobotInternal->isThread()) {

      // a global may execute a set property
      // API TEA program from within the robot thread
      // in this case, API TEA program termination is automatic
      m_pRobotInternal->m_pcVMM[aROBOT_APITEA_PROPPROCID]->
        withinThreadLaunch(
          m_nLinkID,
          m_pSetCUP,
          m_ulSetSize,
          cInputData,
          acpRobotUserProperty::m_vmExitProc,
          this);

    } else {  

      // user executes a set property program
      // from outside the robot thread
      m_pRobotInternal->m_pcVMM[aROBOT_APITEA_PROPPROCID]->
        launchProcess(m_nLinkID,
		      m_pSetCUP,
		      m_ulSetSize,
		      cInputData,
		      acpRobotUserProperty::m_vmExitProc,
		      this);

      // exit proc will be called from robot thread
      // and it will tell user app when it is done
      while (!m_bDone) {
        aIO_MSSleep(m_pRobotInternal->m_ioRef, 1, NULL);
      }
    }

  } else {

    // no TEA program so just call base class method
    acpProperty::setValue(pValue);
    m_bDone = true;
  }
}


/////////////////////////////////////////////////////////////////////
// get implemented with TEA script
//

void acpRobotUserProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  // set up state for when we enter static exit callback
  m_bSet = false;
  m_pValue = pValue;
  m_bDone = false;

  if (m_pGetCUP) {

    // user executes a get property program
    // from outside the robot thread
    m_pRobotInternal->m_pcVMM[aROBOT_APITEA_PROPPROCID]->
      launchProcess(m_nLinkID,
		    m_pGetCUP,
		    m_ulGetSize,
		    NULL,
		    acpRobotUserProperty::m_vmExitProc,
		    this);

    // exit proc will be called from robot thread
    // it will tell us we are done
    while (!m_bDone) {
      aIO_MSSleep(m_pRobotInternal->m_ioRef, 1, NULL);
    }

  } else {

    // no TEA program so just call base class method
    acpProperty::getValue(pObject, pValue);
    m_bDone = true;
  }
}


/////////////////////////////////////////////////////////////////////
// exit callback specific to property
//

aErr acpRobotUserProperty::m_vmExitProc(
  const aVMExit eExitCode,
  const char* returnData,
  const unsigned char returnDataSize,
  const aTEAProcessID pid,
  const void* ref
)
{
  aAssert(pid == aROBOT_APITEA_PROPPROCID);

  // retrieve identity
  acpRobotUserProperty* pProp =
    (acpRobotUserProperty*)ref;

  // get machinery
  acpRobotInternal* pBot =
    pProp->m_pRobotInternal;
  acpRobotVMManager* pcVMM =
    pBot->m_pcVMM[aROBOT_APITEA_PROPPROCID];

  if (eExitCode != aVMExitNormal) {

    // something bad happened while executing a VM opcode
    char msg[aFILE_NAMEMAXCHARS];
    if (pProp->m_bSet) {
      aStringCopy(msg, "API TEA error, set property ");
    } else {
      aStringCopy(msg, "API TEA error, get property ");
    }
    aStringCat(msg, pProp->getName());
    pBot->logMessage(pProp->m_nLinkID, (int)eExitCode, msg);

  } else {

    if (pProp->m_bSet) {

      // nothing to do if this is a set operation

    } else {

      // this conversion work
      // is done in the robot thread

      char* pSource = pcVMM->getRxDataPtr(pProp->m_tGetBlock.cParamIndex);
      pProp->convertOutputToVal(pProp->m_pValue, pSource);
      pProp->m_pValue = NULL;
    }
  }
  // property has been idling
  // so we set the done flag to tell it to stop
  // *** SHOULD NOT TOUCH PROPERTY AFTER THIS ***
  pProp->m_bDone = true;
  pcVMM->setPropDataPtr(NULL, 0);
  pcVMM->setNotActive();
  return aErrNone;
}
