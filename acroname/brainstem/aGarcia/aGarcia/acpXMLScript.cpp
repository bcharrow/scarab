/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXMLScript.cpp                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API action object.    //
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

#include "acpXMLScript.h"
#include "acpGarcia.h"
#include "aGarciaDefs.tea"
#include "aUtil.h"
#include "acpBehaviorList.h"



/////////////////////////////////////////////////////////////////////

acpXMLScript::acpXMLScript() :
  acpObject("xml-script"),
  m_pGarciaInternal(NULL),
  m_ioRef(NULL),
  m_streamInput(NULL),
  m_streamStatus(NULL),
  m_streamError(NULL),
  m_xml(NULL),
  m_pParseBehavior(NULL),
  m_pBehaviorList(NULL),
  m_cCacheName(NULL),
  m_cFileName(NULL),
  m_bParseOkay(true),
  m_bResult(true),
  m_nCtrlFlags(0),
  m_nSignFac(1)
{
}

acpXMLScript::acpXMLScript(
  acpBehaviorList* pBehaviorList,
  acpGarciaInternal* pGarciaInternal,
  aStreamRef input,
  aStreamRef status,
  aStreamRef error
) :
  acpObject("xml-script"),
  m_pGarciaInternal(pGarciaInternal),
  m_ioRef(m_pGarciaInternal->m_ioRef),
  m_streamInput(input),
  m_streamStatus(status),
  m_streamError(error),
  m_xml(NULL),
  m_pParseBehavior(NULL),
  m_pBehaviorList(pBehaviorList),
  m_cCacheName(NULL),
  m_cFileName(NULL),
  m_bParseOkay(true),
  m_bResult(true),
  m_nCtrlFlags(0),
  m_nSignFac(1)
{
}


/////////////////////////////////////////////////////////////////////

acpXMLScript::~acpXMLScript()
{
  aErr actErr = aErrNone;

  /* clean up the XML object */
  if (actErr == aErrNone)
    aXML_Destroy(m_ioRef, m_xml, &actErr);
}


/////////////////////////////////////////////////////////////////////

aErr acpXMLScript::errProc(
  tkError error,
  const unsigned int nLine,
  const unsigned int nColumn,
  const unsigned int nData,
  const char* data[],
  void* errProcRef)
{
  char buff[80];
  acpXMLScript* pal = (acpXMLScript*)errProcRef;
  aStreamRef stream = pal->m_streamError;
  aIOLib ioRef = pal->m_ioRef;
  
  aStringCopy(buff, "ERROR!  file: ");
  aStream_Write(ioRef, stream, buff, aStringLen(buff), NULL);
  aStringCopy(buff, pal->m_cFileName);
  aStream_Write(ioRef, stream, buff, aStringLen(buff), NULL);
  aStringCopy(buff, "  line ");
  aStream_Write(ioRef, stream, buff, aStringLen(buff), NULL);
  aStringFromInt(buff, nLine);
  aStream_Write(ioRef, stream, buff, aStringLen(buff), NULL);
  aStringCopy(buff, ", column ");
  aStream_Write(ioRef, stream, buff, aStringLen(buff), NULL);
  aStringFromInt(buff, nColumn);
  aStream_Write(ioRef, stream, buff, aStringLen(buff), NULL);
  aStream_WriteLine(ioRef, stream, ".", NULL);
  pal->m_bParseOkay = false;
  return aErrNone;
}


/////////////////////////////////////////////////////////////////////

aErr acpXMLScript::sActStart(
  aXMLNodeRef node,
  const char* pKey,
  void* vpRef)
{
  aErr actErr = aErrNone;
  aErr seekErr = aErrNone;
  
  acpXMLScript* pal = (acpXMLScript*)vpRef;
  aAssert(pal);

  // once something has failed there is
  // no need to do any more processing
  if (pal->m_bResult) {

    if (!aStringCompare(pKey, "BEHAVIOR")) {
      if (pal->checkHistory(0)) {

        acpValue valuePrimitiveType;
        acpValue valueBehaviorName;

        // primitive string MUST be found lower in tree
        actErr = pal->xmlSeek("PRIMITIVE", acpValue::kString, node);

        if (actErr == aErrNone)
          valuePrimitiveType.set(pal->m_valueSeek.getStringVal());

        // primitive name string MIGHT be found lower in tree
        if (actErr == aErrNone) {
          seekErr = pal->xmlSeek("NAME", acpValue::kString, node);
          if (seekErr == aErrNotFound) {
            valueBehaviorName.set("");
          } else {
            valueBehaviorName.set(pal->m_valueSeek.getStringVal());
          }
        }

        // try to build behavior and don't set its parent yet
        if (actErr == aErrNone) {
          pal->m_pParseBehavior = 
            pal->m_pGarciaInternal->createNamedBehavior(
              valuePrimitiveType.getStringVal(),
              valueBehaviorName.getStringVal());
        }

        pal->m_nCtrlFlags |= acpACTION_IN_BEHAVIOR;

      } else {
        // *** FOUND AT WRONG LEVEL ***
        pal->runtimeError();
      }
    }

    if (!aStringCompare(pKey, "PROPERTY")) {
      if (pal->checkHistory(acpACTION_IN_BEHAVIOR)) {

        pal->m_nCtrlFlags |= acpACTION_IN_PROPERTY;

      } else {
        // *** FOUND AT WRONG LEVEL ***
        pal->runtimeError();
      }
    }
  }

  return actErr;
}


/////////////////////////////////////////////////////////////////////

aErr acpXMLScript::sActContent(
  aXMLNodeRef node,
  const char* pKey,
  const aToken* pValue,
  void* vpRef)
{
  acpXMLScript* pal = (acpXMLScript*)vpRef;
  aAssert(pal);

  // once something has failed there is
  // no need to do any more processing
  if (pal->m_bResult) {

    if (!aStringCompare(pKey, "NAME")) {
      if (pValue->eType == tkString) {
        pal->m_cCacheName = pValue->v.string;
      }
    }

    if (!aStringCompare(pKey, "VALUE")) {

      if (pal->m_cCacheName) {

        // do hack to handle negative constants
        // store -1 sign fac if special char = '-'
        // otherwise always reset sign fac to +1
        acpValue val;
        switch (pValue->eType) {
          case tkInt:
          {
            val.set(pal->m_nSignFac * pValue->v.integer);
            pal->m_pParseBehavior->setNamedValue(pal->m_cCacheName, &val);
            pal->m_nSignFac = 1;
            break;
          }
          case tkFloat:
          {
            val.set(pal->m_nSignFac * pValue->v.floatVal);
            pal->m_pParseBehavior->setNamedValue(pal->m_cCacheName, &val);
            pal->m_nSignFac = 1;
            break;
          }
          case tkString:
          {
            val.set(pValue->v.string);
            pal->m_pParseBehavior->setNamedValue(pal->m_cCacheName, &val);
            pal->m_nSignFac = 1;
            break;
          }
          case tkSpecial:
            if (pValue->v.special == '-') pal->m_nSignFac = -1;
            break;
        }
      } else {
        // *** NO NAME FOUND ***
        pal->runtimeError();
      }
    }
  }
  return aErrNone;
}


/////////////////////////////////////////////////////////////////////

aErr acpXMLScript::sActEnd(
  aXMLNodeRef node,
  aXMLNodeRef parent,
  const char* pKey,
  void* vpRef)
{
  acpXMLScript* pal = (acpXMLScript*)vpRef;
  aAssert(pal);

  // once something has failed there is
  // no need to do any more processing
  if (pal->m_bResult) {

    // undo history of whatever we're mucking with
    if (!aStringCompare(pKey, "BEHAVIOR"))
      pal->m_nCtrlFlags &= ~acpACTION_IN_BEHAVIOR;
    if (!aStringCompare(pKey, "PROPERTY"))
      pal->m_nCtrlFlags &= ~acpACTION_IN_PROPERTY;

    // when these are done clear the name cache
    if (!aStringCompare(pKey, "PROPERTY") ||
        !aStringCompare(pKey, "BEHAVIOR")) {
      pal->m_cCacheName = NULL;
    }

    // when behavior is completely built
    // add it to script behavior list
    if (!aStringCompare(pKey, "BEHAVIOR"))
      pal->m_pBehaviorList->addChildBehavior(pal->m_pParseBehavior);
  }
  
  return aErrNone;
}


/////////////////////////////////////////////////////////////////////

aErr acpXMLScript::sActSeekContent(
  aXMLNodeRef node,
  const char* pKey,
  const aToken* pValue,
  void* vpRef)
{
  acpXMLScript* pal = (acpXMLScript*)vpRef;
  aAssert(pal);

  switch (pValue->eType) {
    case tkInt:
      pal->m_valueSeek.set(pValue->v.integer);
      break;
    case tkFloat:
      pal->m_valueSeek.set(pValue->v.floatVal);
      break;
    case tkString:
      pal->m_valueSeek.set(pValue->v.string);
      break;
  }

  pal->m_bSeekResult = true;

  return aErrNone;
}


/////////////////////////////////////////////////////////////////////

aErr acpXMLScript::xmlSeek(
  const char* pKey,
  acpValue::eType type,
  aXMLNodeRef node)
{
  aErr actErr = aErrNone;

  // callback will set flag and value if key found
  m_valueSeek.set("");
  m_bSeekResult = false;

  // seek tagged value lower in tree
  aXMLNode_FindKey(m_ioRef,
		   node,
		   pKey,
		   sActSeekContent,
		   this,
		   &actErr);

  // see if we found what we are looking for
  if (!m_bSeekResult) actErr = aErrNotFound;
  if (type != m_valueSeek.getType()) actErr = aErrParam;

  return actErr;
}


/////////////////////////////////////////////////////////////////////

aErr acpXMLScript::xmlImport(
  aStreamRef xmlStream,
  aStreamRef status,
  aStreamRef error)
{
  aErr actErr = aErrNone;
 
  if (!m_ioRef || !m_pGarciaInternal)
    actErr = aErrInitialization;

  // must initialize these before we start
  m_streamStatus = status;
  m_streamError = error;

  if (actErr == aErrNone) 
    aXML_Create(m_ioRef, xmlStream, errProc, this, &m_xml, &actErr);

  return actErr;
}


/////////////////////////////////////////////////////////////////////

bool acpXMLScript::go()
{
  aErr actErr = aErrNone;

  if (m_bParseOkay) {

    if (m_streamStatus)
      aStream_WriteLine(m_ioRef, m_streamStatus, 
      			"Parsing XML Script", NULL);

    // traverse XML tree to perform action
    aXMLCallbacks cb;
    cb.handleStart = sActStart;
    cb.handleContent = sActContent;
    cb.handleEnd = sActEnd;
    cb.ref = this;
    aXML_Traverse(m_ioRef, m_xml, &cb, &actErr);

  } else {
    m_bResult = false;
  }

  if (actErr == aErrNone) {
    if (m_streamStatus) {
      if (m_bResult) {
        aStream_WriteLine(m_ioRef, m_streamStatus, "Parse Okay!", NULL);
      } else {
        aStream_WriteLine(m_ioRef, m_streamStatus, "Parse failed.", NULL);
      }
    }
  }

  // final result will be determined during traversal
  // - will be false if XML has errors
  // - will be false if content has errors
  return m_bResult;
}


/////////////////////////////////////////////////////////////////////

void acpXMLScript::runtimeError()
{
  aStream_WriteLine(m_ioRef, m_streamError, "Runtime Error.", NULL);
  m_bResult = false;
}


/////////////////////////////////////////////////////////////////////

bool acpXMLScript::checkHistory(int nFlags)
{
  if (m_nCtrlFlags == nFlags) return true;
  return false;
}
