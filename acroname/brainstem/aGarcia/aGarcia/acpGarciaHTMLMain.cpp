/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaHTMLMain.h                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of main Garcia API View HTML page.      //
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


#include "aVersion.h"
#include "acpGarciaHTMLMain.h"
#include "acpBehavior.h"



/////////////////////////////////////////////////////////////////////
// acpGarciaHTMLMain requestCB method
//

aErr acpGarciaHTMLMain::requestCB(
  const unsigned int nParamIndex,
  const unsigned int nBlockIndex
)
{
  aErr err = aErrNone;

  switch (nBlockIndex) {
  
  case 0:
    switch (nParamIndex) {

    case 0:
      addHTML(aVERSION_MAJOR);
      break;

    case 1:
      addHTML((int)aVERSION_MINOR);
      break;

    case 2:
      addHTML(aGARCIA_BUILD_NUM);
      break;
    
    case 3:
      if (m_pcGarcia->isActive())
        addHTML("active");
      else
        addHTML("inactive");
      break;

    } // nParamIndex switch
    break;

  case 1:
    switch (nParamIndex) {
    case 0:
      if (m_pcGarcia->m_behaviors.length()) {
        if (!m_pcBehaviorIterator)
          m_pcBehaviorIterator = 
      	    new acpListIterator<acpBehavior>(
      	      m_pcGarcia->m_behaviors);
        aAssert(m_pcBehaviorIterator);
        m_pcCurBehavior = m_pcBehaviorIterator->next();
        if (m_pcCurBehavior) {
          acpTextBuffer buffer(m_ioRef);
          m_pcCurBehavior->getDescription(buffer);
          addHTML(buffer.getBuffer());
        }
        else {
          err = aErrEOF;
          delete m_pcBehaviorIterator;
          m_pcBehaviorIterator = NULL;
          m_pcCurBehavior = NULL;
        }
      } else {
        addHTML("No Current Behaviors");
        err = aErrEOF;
      }
      break;
    } // nParamIndex
    break;

  default:
    break;
  } // switch
  
  return err;

} // acpGarciaHTMLMain requestCB method



/////////////////////////////////////////////////////////////////////
// acpGarciaHTMLMain requestCB method
//

aErr acpGarciaHTMLPrimitives::requestCB(
  const unsigned int nParamIndex,
  const unsigned int nBlockIndex
)
{
  aErr err = aErrNone;

  switch (nBlockIndex) {

  case 0:
    switch (nParamIndex) {
    case 0:
      addHTML(aVERSION_MAJOR);
      break;
    case 1:
      addHTML((int)aVERSION_MINOR);
      break;
    case 2:
      addHTML(aGARCIA_BUILD_NUM);
      break;
    } // nParamIndex switch
    break;

  case 1:
    switch (nParamIndex) {

    case 0:

      // this signals the start of the listing
      if (m_nObjectIndex == -1)
        m_nObjectIndex = 0;

      // move through the sub objects looking for the next primitive
      do {
        m_pCurObject = m_pcGarcia->getSubObject(m_nObjectIndex++);
      } while (m_pCurObject
      	       && (aStringCompare("primitive", 
      	       			  m_pCurObject->getNamedValue("classname")->getStringVal())));
        
      if (m_pCurObject)
        addHTML(m_pCurObject->getNamedValue("name")->getStringVal());
      else {
        if (m_nObjectIndex == 0)
          addHTML("No Current Primitives");
        err = aErrEOF;
        m_nObjectIndex = -1;
        m_pCurObject = NULL;
      }
      break;

    case 1:
      if (m_pCurObject) {
        acpGarciaPrimitive* pPrimitive = (acpGarciaPrimitive*)m_pCurObject;
        addHTML((int)pPrimitive->getCodeSize());
      } else
        err = aErrEOF;
      break;

    case 2:
      if (m_pCurObject) {
        acpGarciaPrimitive* pPrimitive = (acpGarciaPrimitive*)m_pCurObject;
        addHTML((int)pPrimitive->getSlot());
      } else
        err = aErrEOF;
      break;

    case 3:
      if (m_pCurObject)
        addHTML(m_pCurObject->getNamedValue("name")->getStringVal());
      else
        err = aErrEOF;
      break;

    } // switch
    break;

  } // switch
  
  return err;

} // acpGarciaInternalHTMLPrimitiveList requestCB method




/////////////////////////////////////////////////////////////////////
// acpGarciaInternalHTMLPrimitiveParams requestCB method
//

aErr acpGarciaHTMLPrimitiveParams::requestCB (
  const unsigned int nParamIndex,
  const unsigned int nBlockIndex
)
{
  aErr err = aErrNone;

  switch (nBlockIndex) {

  case 0:
    switch (nParamIndex) {

    case 0:
      addHTML(aVERSION_MAJOR);
      break;

    case 1:
      addHTML((int)aVERSION_MINOR);
      break;

    case 2:
      addHTML(aGARCIA_BUILD_NUM);
      break;

    case 3:
      {
        const char* pPrimitiveName = getStringParam("name");
        m_pcPrimitive =
          (acpGarciaPrimitive*)m_pcGarcia->getNamedSubObject("primitive", pPrimitiveName);
        addHTML(pPrimitiveName);
      }
      break;

    case 4:
      aAssert(m_pcPrimitive);
      addHTML(m_pcPrimitive->getBasicPrimitiveHTML());
      addHTML("<BR>");
      addHTML(m_pcPrimitive->getParamHTML());
      m_pcPrimitive = NULL;
      break;

    } // nParamIndex switch
    break;

  case 1:
    break;

  } // switch

  return err;

} // acpGarciaInternalHTMLPrimitiveParams requestCB method



/////////////////////////////////////////////////////////////////////
// acpGarciaInternalHTMLAddBehavior requestCB method
//

aErr acpGarciaHTMLAddBehavior::requestCB (
  const unsigned int nParamIndex,
  const unsigned int nBlockIndex
)
{
  aErr err = aErrNone;

  switch (nBlockIndex) {

  case 0:
    switch (nParamIndex) {

    case 0:
      addHTML(aVERSION_MAJOR);
      break;

    case 1:
      addHTML((int)aVERSION_MINOR);
      break;

    case 2:
      addHTML(aGARCIA_BUILD_NUM);
      break;

    case 3:
      {
        const char* pPrimitiveType = getStringParam("type");
        m_pcPrimitive =
          (acpGarciaPrimitive*)m_pcGarcia->getNamedSubObject("primitive", pPrimitiveType);
        addHTML(pPrimitiveType);
        acpBehavior* pBehavior = m_pcPrimitive->factoryBehavior(*this);
        m_pcGarcia->queueBehavior(pBehavior);
      }
      break;

    case 4:
      aAssert(m_pcPrimitive);
      addHTML(m_pcPrimitive->getParamHTML());
      m_pcPrimitive = NULL;
      break;

    } // nParamIndex switch
    break;

  case 1:
    break;

  } // switch

  return err;

} // acpGarciaHTMLAddBehavior requestCB method
