/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpBehaviorList.cpp                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API behavior list     //
//              object.                                            //
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
#include "acpXMLScript.h"



/////////////////////////////////////////////////////////////////////

acpBehaviorList::acpBehaviorList (
  const char* pName,
  acpGarciaPrimitive* pPrimitive,
  const int nID,
  acpGarciaInternal* pcGarciaInternal,
  acpBehaviorList* pParent // = NULL
) :
  acpBehavior(pName, pPrimitive, nID, pcGarciaInternal, pParent),
  m_nRefCount(0),
  m_nFinalStatus(aGARCIA_ERRFLAG_NOTEXECUTED)
{
}


/////////////////////////////////////////////////////////////////////

acpBehaviorList::~acpBehaviorList()
{
  aAssert(m_nRefCount >= 0);
 
  // children should be gone by now  
  aAssert(m_childBehaviors.isEmpty());
}


/////////////////////////////////////////////////////////////////////

void acpBehaviorList::setNamedValue (
  const char* pPropertyName,
  const acpValue* pValue
)
{
  setValue(getPropertyIndex(pPropertyName), pValue);

} // acpBehaviorList setValue method



/////////////////////////////////////////////////////////////////////

void acpBehaviorList::setValue (
  const int nPropIndex,
  const acpValue* pValue
)
{
  // in a behavior list we need to do some extra work...
  // when a filename is set, we parse the file as a list of 
  // sub-behaviors in XML format
  if (nPropIndex == getPropertyIndex("filename")) {

    aErr err = aErrNone;
    aStreamRef input = NULL;
    aStreamRef status = NULL;
    aStreamRef error = NULL;

    // open XML file
    aStream_CreateFileInput(
      m_pcGarciaInternal->m_ioRef,
      pValue->getStringVal(),
      aFileAreaUser,
      &input,
      &err);

    if (err == aErrNone) {
      status = m_pcGarciaInternal->getNamedValue("status-stream")->getVoidPtrVal();
      error = m_pcGarciaInternal->getNamedValue("error-stream")->getVoidPtrVal();

      acpXMLScript actionList(this,
			      m_pcGarciaInternal,
			      input,
			      status,
			      error);

      if (err == aErrNone)
        err = actionList.xmlImport(input, status, error);

      // an IO or XML format error will prevent queueing  
      if (err == aErrNone)
        actionList.go();
    }
    
    if (input)
      aStream_Destroy(m_pcGarciaInternal->m_ioRef, input, &err);
  }

  // always call base class function
  acpBehavior::setValue(nPropIndex, pValue);

} // acpBehaviorList setValue method



/////////////////////////////////////////////////////////////////////

void acpBehaviorList::addChildBehavior (
  acpBehavior* pChild
)
{
  // children must point back to parent
  if (pChild) {
    pChild->m_pParent = this;
  }

  m_childBehaviors.addToTail(pChild);

} // acpBehaviorList::addChildBehavior method



/////////////////////////////////////////////////////////////////////

void acpBehaviorList::dyingChild (
  const short status,
  bool bSuccess
)
{
  m_nRefCount--;

  aAssert(m_nRefCount >= 0);

  if (!bSuccess) {

    // a child was unsuccessful
    // enraged parent will kill all other siblings
    // they will be deleted from queue and finished off
    while (m_pcGarciaInternal->m_behaviors.head()) {
      acpBehavior* pBehavior;
      m_pcGarciaInternal->m_pcBehaviorMutex->lock();
      pBehavior = m_pcGarciaInternal->m_behaviors.head();
      m_pcGarciaInternal->m_pcBehaviorMutex->unlock();
      if (pBehavior->m_pParent == this) {
        m_pcGarciaInternal->m_behaviors.removeHead();
        pBehavior->m_bNormalTermination = false;
        pBehavior->finish(aGARCIA_ERRFLAG_WONTEXECUTE, NULL);
      } else {
        break;
      }
    }

    // parent then kills self in shame
    // its end status is status of failed child
    finish(status, NULL);

  } else if (m_nRefCount == 0) {

    // all children were successful
    // parent dies happy and fulfilled
    // its end status is status of last succesful child
    finish(status, NULL);
  }

} // acpBehaviorList::dyingChild method



/////////////////////////////////////////////////////////////////////

void acpBehaviorList::execute()
{
  aAssert(m_pPrimitive);

  acpCallback* pcExecuteCB = 
  	getNamedValue(aGARCIA_PROPNAME_EXECCALLBACK)->getCallbackPtrVal();

  if (pcExecuteCB)
    m_pcGarciaInternal->addCallerMessage(
    	new acpCallbackMessage(m_pcGarciaInternal,
    			       this,
    			       pcExecuteCB));

  m_pPrimitive->execute(this);
  m_bRunning = true;

} // execute method



/////////////////////////////////////////////////////////////////////
// acpBehaviorList finish method
// 
// This method is always called within the garcia object's main
// thread context.  We need to get rid of the children in
// this thread context.  Children in a script do not have
// callbacks since you can't define such a thing in a script file.
// If script won't execute then children must be killed here.
// Children who die abnormal death will not
// call the dyingChild method.
//
// Finally do the normal base class finish routine.
//
// Returns true if the behavior remains as a corpse for some 
// other thread to clean up.

bool acpBehaviorList::finish(const short status, bool* pbSuccess)
{
  aAssert(m_pcGarciaInternal->isThread());

  if (status == aGARCIA_ERRFLAG_WONTEXECUTE) {
    while (m_childBehaviors.head()) {
      acpBehavior* pChild;
      pChild = m_childBehaviors.removeTail();
      pChild->m_bNormalTermination = false;
      pChild->finish(aGARCIA_ERRFLAG_WONTEXECUTE, NULL);
    }
  }

  return acpBehavior::finish(status, pbSuccess);
  
} // finish method
