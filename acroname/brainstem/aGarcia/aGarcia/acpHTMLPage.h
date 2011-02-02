/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpHTMLPage.h                                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of an abstract HTML page.               //
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


#ifndef _acpHTMLPage_H_
#define _acpHTMLPage_H_

#include "aUI.h"
#include "acpString.h"

class acpHTMLPage 
{
  public:
  			acpHTMLPage(
  			  const char* pTemplateName = NULL);
    virtual		~acpHTMLPage();
    
    virtual aErr	handleRequest(
			  aStreamRef reply);

    inline void		setup(aIOLib ioRef, 
    			      aUILib uiRef, 
    			      aHTTPRef http)
    			 { m_ioRef = ioRef;
    			   m_uiRef = uiRef; 
    			   m_http = http; }

    const char*		getStringParam(const char* pKey);
    const int		getIntParam(const char* pKey);
    const float		getFloatParam(const char* pKey);

  protected:
    aIOLib		m_ioRef;
    aUILib		m_uiRef;
    aHTTPRef		m_http;
    aStreamRef		m_reply;
    aSymbolTableRef	m_params;
    acpString		m_templateName;

    virtual aErr	requestCB(const unsigned int nParamIndex,
    				  const unsigned int nBlockIndex);

    void		addHTML(const char* pText);
    void		addHTML(const int nVal);

  private:  	
    static aErr 	sTemplateProc(const unsigned int nParamIndex,
				      const unsigned int nBlockIndex,
				      aStreamRef reply,
				      void* vpRef);

  friend class          acpHTMLSite;
};

#endif /* _acpHTMLPage_H_ */
