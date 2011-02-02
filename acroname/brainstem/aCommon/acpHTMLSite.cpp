/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpHTMLSite.cpp                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of a collection of HTML pages used  //
//              with the aHTTP server.                             //
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


#include "acpHTMLSite.h"


/////////////////////////////////////////////////////////////////////
// acpHTMLSite constructor
//

acpHTMLSite::acpHTMLSite(
  aIOLib ioRef,
  aUILib uiRef,
  aSettingFileRef settings
) :
  m_ioRef(ioRef),
  m_uiRef(uiRef)
{
  aErr err;
  aAssert(m_ioRef);
  aAssert(m_uiRef);

  // build the actual http server for this site  
  aHTTP_Create(m_uiRef,
	       settings,
	       handleRequest, 
	       this,
	       &m_http, 
	       &err);
  aAssert(err == aErrNone);

  aSymbolTable_Create(m_ioRef, &m_list, &err);

  aAssert(err == aErrNone);

} // acpHTMLSite constructor



/////////////////////////////////////////////////////////////////////
// acpHTMLSite destructor
//

acpHTMLSite::~acpHTMLSite()
{
  aErr err;

  aHTTP_Destroy(m_uiRef, m_http, NULL);

  aSymbolTable_Destroy(m_ioRef, m_list, &err);

} // acpHTMLSite destructor



/////////////////////////////////////////////////////////////////////
// acpHTMLSite addPage method
//

void acpHTMLSite::addPage(
  const char* pURL,
  acpHTMLPage* pPage
)
{
  aErr err;

  pPage->setup(m_ioRef, m_uiRef, m_http);

  aSymbolTable_Insert(m_ioRef, 
		      m_list,
		      pURL,
		      pPage,
		      deletePage,
		      this,
		      &err);

  aAssert(err == aErrNone);
  		          
} // acpHTMLSite addPage method



/////////////////////////////////////////////////////////////////////
// acpHTMLSite slice method
//

void acpHTMLSite::slice(aBool& bChanged)
{
  aErr err;
  
  aHTTP_TimeSlice(m_uiRef, m_http, &bChanged, &err);

  // This fails with and aErrUnknown on windows but things seem
  // to keep plugging along.  Need to figure out why and then
  // re-enable the assertion.
  aAssert(err == aErrNone);

} // acpHTMLSite slice method



/////////////////////////////////////////////////////////////////////
// acpHTMLSite deletePage static method
//

aErr acpHTMLSite::deletePage (
  void* pData, 
  void* ref
)
{
  acpHTMLPage* pPage = (acpHTMLPage*)pData; 
  delete pPage;
  
  return aErrNone;
  
} // acpHTMLSite deletePage static method



/////////////////////////////////////////////////////////////////////
// acpHTMLSite handleRequest method
//

aErr acpHTMLSite::handleRequest (
  const char* pURL,
  aSymbolTableRef params,
  aStreamRef reply,
  void* vpRef
)
{
  acpHTMLSite* pSite = (acpHTMLSite*)vpRef;

  return pSite->HTTP(pURL, params, reply);

} // acpHTMLSite handleRequest static method



/////////////////////////////////////////////////////////////////////
// acpHTMLSite HTTP method
//

aErr acpHTMLSite::HTTP (
  const char* pURL,
  aSymbolTableRef params,
  aStreamRef reply)
{
  aErr err;
  void* vp;

  aSymbolTable_Find(m_ioRef,
  		    m_list,
  		    pURL,
  		    &vp,
  		    &err);

  if (err == aErrNone) {
    acpHTMLPage* pPage = (acpHTMLPage*)vp;
    pPage->m_params = params;
    err = pPage->handleRequest(reply);
    pPage->m_params = NULL;
  }

  return err;

} // acpHTMLSite HTTP method
