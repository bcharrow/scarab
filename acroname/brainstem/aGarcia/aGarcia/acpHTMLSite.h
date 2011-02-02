/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpHTMLSite.h                                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of a collection of HTML pages used with //
//              the aHTTP server.                                  //
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


#ifndef _acpHTMLSite_H_
#define _acpHTMLSite_H_

#include "aIO.h"
#include "acpHTMLPage.h"

class acpHTMLSite
{
  public:
  			acpHTMLSite(aIOLib ioRef,
  				    aUILib uiRef,
  				    aSettingFileRef settings);
    virtual		~acpHTMLSite();

    void		addPage(const char* pURL,
    			        acpHTMLPage* pPage);

    void		slice(aBool& bChanged);

  private:
    static aErr		handleRequest(const char* pURL,
    				      aSymbolTableRef params,
    				      aStreamRef reply,
    				      void* vpRef);
    static aErr         deletePage(void* pData, void* ref);
    aErr 		HTTP(const char* pURL,
			     aSymbolTableRef params,
			     aStreamRef reply);

    aIOLib		m_ioRef;
    aUILib		m_uiRef;
    aHTTPRef		m_http;
    aSymbolTableRef	m_list;
};

#endif // _acpHTMLSite_H_
