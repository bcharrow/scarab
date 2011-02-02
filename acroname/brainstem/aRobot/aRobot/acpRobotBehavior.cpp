/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotBehavior.cpp                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API behavior object.   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//  Behaviors parametrize primitives.  They can also act as        //
//  containers for other sets of behaviors.                        //
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

#include "aRobotDefs.tea"

#include "acpRobotBehaviorList.h"
#include "aRobotProperties.h"
#include "acpRobotInternal.h"
#include "acpRobotCallbackMessage.h"



/////////////////////////////////////////////////////////////////////

acpRobotBehavior::acpRobotBehavior (
  const char* pName,
  acpRobotPrimitive* pPrimitive,
  const int nID,
  acpRobotInternal* pcRobotInternal,
  acpRobotBehaviorList* pParent
) :
  acpObject("behavior", pName),
  m_pParent(pParent),
  m_bNormalTermination(true),
  m_nFinalStatus(aROBOT_ERRFLAG_NOTEXECUTED),
  m_pcRobotInternal(pcRobotInternal),
  m_pPrimitive(pPrimitive),
  m_bRunning(false),
  m_bResult(true)
{
  // behaviors should clone the base primitive's properties
  // except the classname and name
  pPrimitive->enumProperties(propertyCloneProc, this);
  
  // now, go set the unique id
  acpValue uniqueID(nID);
  setNamedValue(aROBOT_PROPNAME_UNIQUEID, &uniqueID);
}



/////////////////////////////////////////////////////////////////////
// acpRobotBehavior destructor
//
// This could be called from any thread, be careful not to manipulate
// anything in the garcia internal object without considering threads
//
// This destructor is only called via the finish method or in rare 
// cases by the behavior list when the garcia object is being torn
// down with remaining behaviors.

acpRobotBehavior::~acpRobotBehavior()
{
  // null indicates a non-existent callback
  // or a callback that has already been called
  // so non-null callbacks can be safely destroyed
  acpCallback* pCB;
  pCB = getNamedValue(aROBOT_PROPNAME_COMPCALLBACK)->getCallbackPtrVal();
  if (pCB)
    delete pCB;
  pCB = getNamedValue(aROBOT_PROPNAME_EXECCALLBACK)->getCallbackPtrVal();
  if (pCB)
    delete pCB;

} // acpRobotBehavior destructor



/////////////////////////////////////////////////////////////////////

aErr acpRobotBehavior::writeToStream(const aStreamRef stream) const
{
  return aErrNone;

} // writeToStream method



/////////////////////////////////////////////////////////////////////

void acpRobotBehavior::setNamedValue (
  const char* pPropertyName,
  const acpValue* pValue
)
{
  if (!aStringCompare(m_pPrimitive->
		        getValue(m_pPrimitive->m_nNamePropertyIndex)->
			  getStringVal(), aROBOT_PRIMNAME_GLOBAL)) {

    // global properties are identified by strings
    // they won't have an index in the behavior
    // so the special global case must be caught here
    int i = m_pcRobotInternal->getPropertyIndex(pPropertyName);

    if (i >= 0) {
      // then it's a property of the main Robot object
      // save it in a list till it's time to execute the global behavior
      // then exit because there is no behavior property to set
      m_globalIndex.add(new acpInt(i));
      m_globalValue.add(new acpValue(pValue));
      return;
    }
  }

  // normal case
  setValue(getPropertyIndex(pPropertyName), pValue);
}



/////////////////////////////////////////////////////////////////////

void acpRobotBehavior::setValue (
  const int nPropIndex,
  const acpValue* pValue
)
{
  // many behaviors have expected status  
  // expected status value requires extra work
  // these are write-only, they get pushed into a member list
  if (nPropIndex == getPropertyIndex(aROBOT_PROPNAME_EXPECTSTATUS)) {
    m_expectedStatus.add(new acpInt(pValue->getIntVal()));
  }

  // always call base class
  acpObject::setValue(nPropIndex, pValue);
}



/////////////////////////////////////////////////////////////////////

void acpRobotBehavior::execute()
{
  aAssert(m_pPrimitive);

  // add some status when requested
  if (m_pcRobotInternal->hasStatus()) {
    char line[100];
    aStringCopy(line, m_pPrimitive->getValue(m_pPrimitive->m_nNamePropertyIndex)->getStringVal());
    aStringCat(line, " named \"");
    aStringCat(line, getNamedValue(aROBOT_PROPNAME_NAME)->getStringVal());
    aStringCat(line, "\" executing");
    m_pcRobotInternal->addStatusLine(line);
  }

  // this is a message to the callback handler
  // that will let it know when to delete self
  // (must delete after performing completion callback)
  m_bRunning = true;

  acpCallback* pcCB = 
    getNamedValue(aROBOT_PROPNAME_EXECCALLBACK)->getCallbackPtrVal();

  if (pcCB) {

    m_pcRobotInternal->addCallerMessage(
    	new acpRobotCallbackMessage(m_pcRobotInternal, this, pcCB, false));

    // callback message has been shipped so null the stored ptr
    acpValue nullcb((acpCallback*)NULL);
    setNamedValue(aROBOT_PROPNAME_EXECCALLBACK, &nullcb);
  }

  m_pPrimitive->execute(this);

} // execute method



/////////////////////////////////////////////////////////////////////
// acpRobotBehavior finish method
// 
// This method is always called within the garcia object's main
// thread context.
//
// Returns true if the behavior remains as a corpse for some 
// other thread to clean up.

bool acpRobotBehavior::finish(const short status, bool* pbSuccess)
{
  aAssert(m_pcRobotInternal->isThread());

  bool bResult = true;

  // we are done and no longer running
  // this lets completion proc (if any) know to delete
  m_bRunning = false;

  // add some status, when requested
  if (m_pcRobotInternal->hasStatus()) {
    char line[100];
    aStringCopy(line, m_pPrimitive->getValue(m_pPrimitive->m_nNamePropertyIndex)->getStringVal());
    aStringCat(line, " named \"");
    aStringCat(line, getNamedValue(aROBOT_PROPNAME_NAME)->getStringVal());
    aStringCat(line, "\" finished");
    m_pcRobotInternal->addStatusLine(line);

    aStringCopy(line, "  status = ");
    char num[10];
    char msg[100];
    m_pcRobotInternal->statusString(status, msg,  100);
    aStringCat(line, msg);
    aStringCat(line, "(");
    aStringFromInt(num, status);
    aStringCat(line, num);
    aStringCat(line, ")");
    m_pcRobotInternal->addStatusLine(line);
  }

  // set how we actually completed for callbacks to access
  acpValue statusVal(status);
  setNamedValue(aROBOT_PROPNAME_COMPSTATUS, &statusVal);

  // check for any expected results
  // if none then check for  normal exit  
  if (m_expectedStatus.isEmpty()) {
  
    bResult = (status == aROBOT_ERRFLAG_NORMAL);

  } else {

    acpListIterator<acpInt> params(m_expectedStatus);
    acpInt* pValue;
    bResult = false;

    while ((pValue = params.next())) {
      if (pValue->m_val == status) {
        bResult = true;
        break;
      }
    }
  }

  // this lets parent know how we did
  m_nFinalStatus = status;
  m_bResult = bResult;
  if (pbSuccess) *pbSuccess = bResult;


  ///////////////////////////////////////////////////////////////
  // Before we clean up the behavior, we need to ensure that
  // the garcia object maintains no references to this behavior
  // object since we are about to delete it.

  // if we have a parent then
  // inform it of our impending death (how sad)
  // children who are killed by parent do not do this
  // because parent already knows they are dying
  // (this prevents circular situations)
  if (m_pParent) {
    if (m_bNormalTermination)
      m_pParent->dyingChild(m_nFinalStatus, m_bResult);
  } else {
    // top level abnormal death requires queue flush
    // behaviors that won't execute are killed by a previous
    // unsuccessful behavior so they don't flush queue
    if (!m_bResult && (status != aROBOT_ERRFLAG_WONTEXECUTE))
      m_pcRobotInternal->flushQueue();
  }

  // remove ourselves as the current behavior
  m_pcRobotInternal->clearCurrent(this);    


  ///////////////////////////////////////////////////////////////
  // Finally, handle disposing of the behavior
  //
  // Here there are two choices that both result in the destruction
  // of the behavior.
  // 1. There is a callback.
  //      In this case, the calling thread is messaged to allow
  //      the callback to be performed on the corpse of the behavior
  //      once the callback has been performed, the corpse is then
  //      put to rest.
  // 2. There is no callback.
  //      In this case, the behavior is deleted right here.

  acpCallback* pcCompletionCB = 
    getNamedValue(aROBOT_PROPNAME_COMPCALLBACK)->getCallbackPtrVal();

  if (pcCompletionCB) {

    m_pcRobotInternal->addCallerMessage(
        new acpRobotCallbackMessage(m_pcRobotInternal,
    			       this,
    			       pcCompletionCB, true));

    // callback message has been shipped so null the stored ptr
    acpValue nullcb((acpCallback*)NULL);
    setNamedValue(aROBOT_PROPNAME_COMPCALLBACK, &nullcb);

    return true;

  } else {

    // now commit suicide
    // which will result in this behavior's death
    delete this;
  }

  return false;

} // finish method



/////////////////////////////////////////////////////////////////////

void acpRobotBehavior::getDescription(acpTextBuffer& buffer)
{
#if 0
  buffer.add(m_pName);

  acpListIterator<acpValue> params(m_params);
  acpValue* pValue;
  int i = 0;

#pragma warn_possunwant off
  pValue = params.next();
  if (pValue) {
    do {
    
      if (i++ == 0)
        buffer.add(": ");

      pValue->getDescription(buffer);

      pValue = params.next();
    
      if (pValue)
        buffer.add(", ");
    } while (pValue);
  }
#pragma warn_possunwant on    
#endif
}


/////////////////////////////////////////////////////////////////////

aErr acpRobotBehavior::propertyCloneProc(
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS flags,
  void* vpRef
)
{
  acpRobotBehavior* pBehavior = (acpRobotBehavior*)vpRef;
  aAssert(pBehavior);

  // we don't need to clone these
  if (nIndex == pBehavior->m_nClassPropertyIndex)
    return aErrNone;
  if (nIndex == pBehavior->m_nNamePropertyIndex)
    return aErrNone;

  pBehavior->addProperty(new acpProperty(pName, flags), 
    			 pBehavior->m_pPrimitive->getValue(nIndex));
  return aErrNone;
}



/////////////////////////////////////////////////////////////////////

acpRobotBehaviorProperty::acpRobotBehaviorProperty (
  acpRobotPrimitive* pPrimitive,
  const char* pName,
  aPROPERTY_FLAGS flags
) :
  acpProperty(pName, flags),
  m_pPrimitive(pPrimitive)
{
}


/////////////////////////////////////////////////////////////////////
// execute callback property
/////////////////////////////////////////////////////////////////////

acpRobotBehaviorExecuteCBProperty::acpRobotBehaviorExecuteCBProperty (
  acpRobotPrimitive* pPrimitive
) :
  acpRobotBehaviorProperty(pPrimitive,
  		      aROBOT_PROPNAME_EXECCALLBACK,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_READ
  		       | aPROPERTY_FLAG_CALLBACK))
{
}


/////////////////////////////////////////////////////////////////////
// completion callback property
/////////////////////////////////////////////////////////////////////

acpRobotBehaviorCompletionCBProperty::acpRobotBehaviorCompletionCBProperty (
  acpRobotPrimitive* pPrimitive
) :
  acpRobotBehaviorProperty(pPrimitive,
  		      aROBOT_PROPNAME_COMPCALLBACK,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_READ
  		       | aPROPERTY_FLAG_CALLBACK))
{
}


/////////////////////////////////////////////////////////////////////
// status property
/////////////////////////////////////////////////////////////////////

acpRobotBehaviorStatusProperty::acpRobotBehaviorStatusProperty (
  acpRobotPrimitive* pPrimitive
) :
  acpRobotBehaviorProperty(pPrimitive,
  		      aROBOT_PROPNAME_COMPSTATUS,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_READ
  		       | aPROPERTY_FLAG_INT))
{
}


/////////////////////////////////////////////////////////////////////
// expected status value property
/////////////////////////////////////////////////////////////////////

acpRobotBehaviorExpectedStatusProperty::acpRobotBehaviorExpectedStatusProperty (
  acpRobotPrimitive* pPrimitive
) :
  acpRobotBehaviorProperty(pPrimitive,
  		      aROBOT_PROPNAME_EXPECTSTATUS,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_INT))
{
}


/////////////////////////////////////////////////////////////////////
// param html value property
/////////////////////////////////////////////////////////////////////

acpRobotBehaviorParamHTMLProperty::acpRobotBehaviorParamHTMLProperty (
  acpRobotPrimitive* pPrimitive
) :
  acpRobotBehaviorProperty(pPrimitive,
  		      aROBOT_PROPNAME_PARAMHTML,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_READ
  		       | aPROPERTY_FLAG_STRING))
{
}
