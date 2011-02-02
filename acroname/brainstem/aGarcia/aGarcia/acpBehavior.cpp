/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpBehavior.cpp                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API behavior object.  //
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

#include "acpBehaviorList.h"
#include "aGarciaProperties.h"
#include "acpGarciaInternal.h"
#include "acpCallbackMessage.h"
#include "aGarciaDefs.tea"


/////////////////////////////////////////////////////////////////////

acpBehavior::acpBehavior (
  const char* pName,
  acpGarciaPrimitive* pPrimitive,
  const int nID,
  acpGarciaInternal* pcGarciaInternal,
  acpBehaviorList* pParent
) :
  acpObject("behavior", pName),
  m_pParent(pParent),
  m_bNormalTermination(true),
  m_nFinalStatus(aGARCIA_ERRFLAG_NOTEXECUTED),
  m_pcGarciaInternal(pcGarciaInternal),
  m_pPrimitive(pPrimitive),
  m_bRunning(false),
  m_bResult(true)
{
  // behaviors should clone the base primitive's properties
  // except the classname and name
  pPrimitive->enumProperties(propertyCloneProc, this);
  
  // now, go set the unique id
  acpValue uniqueID(nID);
  setNamedValue(aGARCIA_PROPNAME_UNIQUEID, &uniqueID);
}



/////////////////////////////////////////////////////////////////////
// acpBehavior destructor
//
// This could be called from any thread, be careful not to manipulate
// anything in the garcia internal object without considering threads
//
// This destructor is only called via the finish method or in rare 
// cases by the behavior list when the garcia object is being torn
// down with remaining behaviors.

acpBehavior::~acpBehavior()
{
  // if a behavior wasn't executed, it could be still holding on
  // to a callback object that should be cleaned up.
  acpCallback* pCB;

  // MRW ???  Does this ever occur?
  // A behavior will issue a completion callback if it won't execute.
  pCB = getNamedValue("completion-callback")->getCallbackPtrVal();
  if (pCB && (m_nFinalStatus == aGARCIA_ERRFLAG_NOTEXECUTED))
    delete pCB;

  pCB = getNamedValue("execute-callback")->getCallbackPtrVal();
  if (pCB &&
      ((m_nFinalStatus == aGARCIA_ERRFLAG_NOTEXECUTED) ||
       (m_nFinalStatus == aGARCIA_ERRFLAG_WONTEXECUTE)))
    delete pCB;

} // acpBehavior destructor



/////////////////////////////////////////////////////////////////////

aErr acpBehavior::writeToStream(const aStreamRef stream) const
{
  return aErrNone;

} // writeToStream method



/////////////////////////////////////////////////////////////////////

void acpBehavior::setNamedValue (
  const char* pPropertyName,
  const acpValue* pValue
)
{
  if (!aStringCompare(m_pPrimitive->getValue(m_pPrimitive->m_nNamePropertyIndex)->getStringVal(), "global")) {

    // global properties are identified by strings
    // they won't have an index in the behavior
    // so the special global case must be caught here
    int i = m_pcGarciaInternal->getPropertyIndex(pPropertyName);

    if (i >= 0) {
      // then it's a property of the main Garcia object
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

void acpBehavior::setValue (
  const int nPropIndex,
  const acpValue* pValue
)
{
  // many behaviors have expected status  
  // expected status value requires extra work
  // these are write-only, they get pushed into a member list
  if (nPropIndex == getPropertyIndex(aGARCIA_PROPNAME_EXPECTSTATUS)) {
    m_expectedStatus.add(new acpInt(pValue->getIntVal()));
  }

  // always call base class
  acpObject::setValue(nPropIndex, pValue);
}



/////////////////////////////////////////////////////////////////////

void acpBehavior::execute()
{
  aAssert(m_pPrimitive);

  // add some status when requested
  if (m_pcGarciaInternal->hasStatus()) {
    char line[100];
    aStringCopy(line, m_pPrimitive->getValue(m_pPrimitive->m_nNamePropertyIndex)->getStringVal());
    aStringCat(line, " named \"");
    aStringCat(line, getNamedValue(aGARCIA_PROPNAME_NAME)->getStringVal());
    aStringCat(line, "\" executing");
    m_pcGarciaInternal->addStatusLine(line);
  }

  // this is a message to the callback handler
  // that will let it know when to delete self
  // (must delete after performing completion callback)
  m_bRunning = true;

  acpCallback* pcCB = 
  	getNamedValue(aGARCIA_PROPNAME_EXECCALLBACK)->getCallbackPtrVal();

  if (pcCB)
    m_pcGarciaInternal->addCallerMessage(
    	new acpCallbackMessage(m_pcGarciaInternal, this, pcCB));

  m_pPrimitive->execute(this);

} // execute method



/////////////////////////////////////////////////////////////////////
// acpBehavior finish method
// 
// This method is always called within the garcia object's main
// thread context.
//
// Returns true if the behavior remains as a corpse for some 
// other thread to clean up.

bool acpBehavior::finish(const short status, bool* pbSuccess)
{
  aAssert(m_pcGarciaInternal->isThread());

  bool bResult = true;

  // we are done and no longer running
  // this lets completion proc (if any) know to delete
  m_bRunning = false;

  // add some status, when requested
  if (m_pcGarciaInternal->hasStatus()) {
    char line[100];
    aStringCopy(line, m_pPrimitive->getValue(m_pPrimitive->m_nNamePropertyIndex)->getStringVal());
    aStringCat(line, " named \"");
    aStringCat(line, getNamedValue(aGARCIA_PROPNAME_NAME)->getStringVal());
    aStringCat(line, "\" finished");
    m_pcGarciaInternal->addStatusLine(line);

    aStringCopy(line, "  status = ");
    char num[10];
    char msg[100];
    m_pcGarciaInternal->statusString(status, msg,  100);
    aStringCat(line, msg);
    aStringCat(line, "(");
    aStringFromInt(num, status);
    aStringCat(line, num);
    aStringCat(line, ")");
    m_pcGarciaInternal->addStatusLine(line);
  }

  // set how we actually completed for callbacks to access
  acpValue statusVal(status);
  setNamedValue(aGARCIA_PROPNAME_COMPSTATUS, &statusVal);

  // check for any expected results
  // if none then check for  normal exit  
  if (m_expectedStatus.isEmpty()) {
  
    bResult = (status == aGARCIA_ERRFLAG_NORMAL);

  } else {

    acpListIterator<acpInt> params(m_expectedStatus);
    acpInt* pValue;
    bResult = false;

    while ((pValue = params.next())) {
      if (pValue->m_val == status)
        bResult = true;
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
    if (!m_bResult && (status != aGARCIA_ERRFLAG_WONTEXECUTE))
      m_pcGarciaInternal->flushQueue();
  }

  // remove ourselves as the current behavior
  m_pcGarciaInternal->clearCurrent(this);    


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
  		getNamedValue(aGARCIA_PROPNAME_COMPCALLBACK)->getCallbackPtrVal();

  if (pcCompletionCB) {
    m_pcGarciaInternal->addCallerMessage(
        new acpCallbackMessage(m_pcGarciaInternal,
    			       this,
    			       pcCompletionCB));
    return true;

  } else {

    // now commit suicide
    // which will result in this behavior's death
    delete this;
  }

  return false;

} // finish method



/////////////////////////////////////////////////////////////////////

void acpBehavior::getDescription(acpTextBuffer& buffer)
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

aErr acpBehavior::propertyCloneProc(
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS flags,
  void* vpRef
)
{
  acpBehavior* pBehavior = (acpBehavior*)vpRef;
  aAssert(pBehavior);

  if (nIndex == pBehavior->m_nClassPropertyIndex)
    return aErrNone;
 
  if (nIndex == pBehavior->m_nNamePropertyIndex)
    return aErrNone;

  pBehavior->addProperty(new acpProperty(pName, flags), 
    			 pBehavior->m_pPrimitive->getValue(nIndex));
  return aErrNone;
}



/////////////////////////////////////////////////////////////////////

acpBehaviorProperty::acpBehaviorProperty (
  acpGarciaPrimitive* pPrimitive,
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

acpBehaviorExecuteCBProperty::acpBehaviorExecuteCBProperty (
  acpGarciaPrimitive* pPrimitive
) :
  acpBehaviorProperty(pPrimitive,
  		      aGARCIA_PROPNAME_EXECCALLBACK,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_READ
  		       | aPROPERTY_FLAG_CALLBACK))
{
}


/////////////////////////////////////////////////////////////////////
// completion callback property
/////////////////////////////////////////////////////////////////////

acpBehaviorCompletionCBProperty::acpBehaviorCompletionCBProperty (
  acpGarciaPrimitive* pPrimitive
) :
  acpBehaviorProperty(pPrimitive,
  		      aGARCIA_PROPNAME_COMPCALLBACK,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_READ
  		       | aPROPERTY_FLAG_CALLBACK))
{
}


/////////////////////////////////////////////////////////////////////
// status property
/////////////////////////////////////////////////////////////////////

acpBehaviorStatusProperty::acpBehaviorStatusProperty (
  acpGarciaPrimitive* pPrimitive
) :
  acpBehaviorProperty(pPrimitive,
  		      aGARCIA_PROPNAME_COMPSTATUS,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_READ
  		       | aPROPERTY_FLAG_INT))
{
}


/////////////////////////////////////////////////////////////////////
// expected status value property
/////////////////////////////////////////////////////////////////////

acpBehaviorExpectedStatusProperty::acpBehaviorExpectedStatusProperty (
  acpGarciaPrimitive* pPrimitive
) :
  acpBehaviorProperty(pPrimitive,
  		      aGARCIA_PROPNAME_EXPECTSTATUS,
  		      (aPROPERTY_FLAG_WRITE
  		       | aPROPERTY_FLAG_INT))
{
}
