/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotBehaviorList.cpp                                  //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API behavior list      //
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

#include "acpRobotBehaviorList.h"
#include "aRobotProperties.h"
#include "acpRobotInternal.h"
#include "acpRobotCallbackMessage.h"
#include "aRobotDefs.tea"
#include "acpRobotXMLReader.h"



/////////////////////////////////////////////////////////////////////

acpRobotBehaviorList::acpRobotBehaviorList (
  const char* pName,
  acpRobotPrimitive* pPrimitive,
  const int nID,
  acpRobotInternal* pcRobotInternal,
  acpRobotBehaviorList* pParent // = NULL
) :
  acpRobotBehavior(pName, pPrimitive, nID, pcRobotInternal, pParent),
  m_nRefCount(0),
  m_nFinalStatus(aROBOT_ERRFLAG_NOTEXECUTED)
{
}


/////////////////////////////////////////////////////////////////////

acpRobotBehaviorList::~acpRobotBehaviorList()
{
  aAssert(m_nRefCount >= 0);
 
  // children should be gone by now  
  aAssert(m_childBehaviors.isEmpty());
}


/////////////////////////////////////////////////////////////////////

void acpRobotBehaviorList::setNamedValue (
  const char* pPropertyName,
  const acpValue* pValue
)
{
  setValue(getPropertyIndex(pPropertyName), pValue);

} // acpRobotBehaviorList setValue method



/////////////////////////////////////////////////////////////////////

void acpRobotBehaviorList::setValue (
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
      m_pcRobotInternal->m_ioRef,
      pValue->getStringVal(),
      aFileAreaUser,
      &input,
      &err);

    if (err == aErrNone) {
      status = m_pcRobotInternal->
        getNamedValue(aROBOT_PROPNAME_STATUSSTREAM)->getVoidPtrVal();
      error = m_pcRobotInternal->
        getNamedValue(aROBOT_PROPNAME_ERRORSTREAM)->getVoidPtrVal();

      acpRobotXMLReader actionList(this,
				   m_pcRobotInternal,
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
      aStream_Destroy(m_pcRobotInternal->m_ioRef, input, &err);
  }

  // always call base class function
  acpRobotBehavior::setValue(nPropIndex, pValue);

} // acpRobotBehaviorList setValue method



/////////////////////////////////////////////////////////////////////

void acpRobotBehaviorList::addChildBehavior (
  acpRobotBehavior* pChild
)
{
  // children must point back to parent
  if (pChild) {
    pChild->m_pParent = this;
  }

  m_childBehaviors.addToTail(pChild);

} // acpRobotBehaviorList::addChildBehavior method



/////////////////////////////////////////////////////////////////////

void acpRobotBehaviorList::dyingChild (
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
    while (m_pcRobotInternal->m_behaviors.head()) {
      acpRobotBehavior* pBehavior;
      m_pcRobotInternal->m_pcBehaviorMutex->lock();
      pBehavior = m_pcRobotInternal->m_behaviors.head();
      m_pcRobotInternal->m_pcBehaviorMutex->unlock();
      if (pBehavior->m_pParent == this) {
        m_pcRobotInternal->m_behaviors.removeHead();
        pBehavior->m_bNormalTermination = false;
        pBehavior->finish(aROBOT_ERRFLAG_WONTEXECUTE, NULL);
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

} // acpRobotBehaviorList::dyingChild method


/////////////////////////////////////////////////////////////////////
// acpRobotBehaviorList finish method
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

bool acpRobotBehaviorList::finish(const short status, bool* pbSuccess)
{
  aAssert(m_pcRobotInternal->isThread());
  if (status == aROBOT_ERRFLAG_WONTEXECUTE) {
    while (m_childBehaviors.head()) {
      acpRobotBehavior* pChild;
      pChild = m_childBehaviors.removeTail();
      pChild->m_bNormalTermination = false;
      pChild->finish(aROBOT_ERRFLAG_WONTEXECUTE, NULL);
    }
  }
  return acpRobotBehavior::finish(status, pbSuccess);
  
} // finish method
