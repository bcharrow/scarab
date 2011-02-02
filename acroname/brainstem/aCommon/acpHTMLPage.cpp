/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpHTMLPage.cpp                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of an abstract HTML page.           //
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

#include "aUtil.h"
#include "acpHTMLPage.h"



/////////////////////////////////////////////////////////////////////
// acpHTMLPage constructor
//

acpHTMLPage::acpHTMLPage(
  const char* pTemplateName
) :
  m_reply(NULL),
  m_params(NULL),
  m_templateName(pTemplateName)
{
} // acpHTMLPage constructor



/////////////////////////////////////////////////////////////////////
// acpHTMLPage destructor
//

acpHTMLPage::~acpHTMLPage()
{
} // acpHTMLPage constructor



/////////////////////////////////////////////////////////////////////
// acpHTMLPage handleRequest method
//

aErr acpHTMLPage::handleRequest(
  aStreamRef reply
)
{
  aErr err = aErrNone;

  if (m_templateName.length()) {
    aHTTP_Template(m_uiRef, 
                   m_http, 
                   m_templateName, 
                   sTemplateProc,
                   this,
                   reply,
                   &err);
  }

  return err;

} // acpHTMLPage handleRequest method



/////////////////////////////////////////////////////////////////////
// sTemplateProc static method
//

aErr acpHTMLPage::sTemplateProc (
  const unsigned int nParamIndex,
  const unsigned int nBlockIndex,
  aStreamRef reply,
  void* vpRef
)
{
  acpHTMLPage* pPage = (acpHTMLPage*)vpRef;

  pPage->m_reply = reply;

  aErr err = pPage->requestCB(nParamIndex, nBlockIndex);

  pPage->m_reply = NULL;

  return err;

} // sTemplateProc static method



/////////////////////////////////////////////////////////////////////
// templateProc virtual method
//

aErr acpHTMLPage::requestCB(const unsigned int nParamIndex,
    		            const unsigned int nBlockIndex)
{
  return aErrNone;

} // acpHTMLPage requestCB method



/////////////////////////////////////////////////////////////////////
// acpHTMLPage addHTML(const char*) method
//

void acpHTMLPage::addHTML(const char* pText)
{
  aAssert(m_reply);

  unsigned int len = aStringLen(pText);
  if (len) {
    aErr err = aErrNone;
    aStream_Write(m_ioRef, m_reply, pText, len, &err);
    aAssert(err == aErrNone);
  }
  
} // acpHTMLPage addHTML(const char*) method



/////////////////////////////////////////////////////////////////////
// acpHTMLPage addHTML(const int) method
//

void acpHTMLPage::addHTML(const int nVal)
{
  aAssert(m_reply);
  char num[16];
  
  aStringFromInt(num, nVal);

  unsigned int len = aStringLen(num);
  if (len) {
    aErr err = aErrNone;
    aStream_Write(m_ioRef, m_reply, num, len, &err);
    aAssert(err == aErrNone);
  }
  
} // acpHTMLPage addHTML(const int) method



/////////////////////////////////////////////////////////////////////
// acpHTMLPage getStringParam method
//

const char* acpHTMLPage::getStringParam(const char* pKey)
{
  const char* pReply = "";

  if (m_params) {
    aErr findErr;
    void* vp;
    if (!aSymbolTable_Find(m_ioRef, m_params, pKey, 
			   &vp, &findErr))
      pReply = (char*)vp;
  }

  return pReply;

} // acpHTMLPage getStringParam method



/////////////////////////////////////////////////////////////////////
// acpHTMLPage getIntParam method
//

const int acpHTMLPage::getIntParam(const char* pKey)
{
  int nVal = 0;
  
  if (m_params) {
    void* vp;;
    aErr findErr;
    
    aSymbolTable_Find(m_ioRef, m_params, pKey, 
		      &vp, &findErr);
    
    if (findErr == aErrNone)
      aIntFromString(&nVal, (char*)vp);
  }

  return nVal;

} // acpHTMLPage getIntParam method



/////////////////////////////////////////////////////////////////////
// acpHTMLPage getFloatParam method
//

const float acpHTMLPage::getFloatParam(const char* pKey)
{
  float fVal = 0;

  if (m_params) {
  
    void* vp;;
    aErr findErr;
    
    aSymbolTable_Find(m_ioRef, m_params, pKey, 
		      &vp, &findErr);
    
    if (findErr == aErrNone)
      aUtil_ParseFloat(&fVal, (char*)vp, &findErr);
  }

  return fVal;

} // acpHTMLPage getFloatParam method
