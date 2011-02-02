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

#ifndef _aGarciaHTMLMain_H_
#define _aGarciaHTMLMain_H_

#include "acpList.h"
#include "acpHTMLPage.h"
#include "acpGarciaInternal.h"
#include "acpGarciaPrimitive.h"


/////////////////////////////////////////////////////////////////////

class acpGarciaHTMLMain :
  public acpHTMLPage
{
  public:
	  		acpGarciaHTMLMain(acpGarciaInternal* pGarcia) : 
	  		  acpHTMLPage("agAPIViewMain.tpl"),
	  		  m_pcGarcia(pGarcia),
	  		  m_pcBehaviorIterator(NULL),
	  		  m_pcCurBehavior(NULL) {}

  protected:
    aErr		requestCB(const unsigned int nParamIndex,
				  const unsigned int nBlockIndex);
    acpGarciaInternal*	m_pcGarcia;
    acpListIterator<acpBehavior>*	
    			m_pcBehaviorIterator;
    acpBehavior* 	m_pcCurBehavior;
};


/////////////////////////////////////////////////////////////////////

class acpGarciaHTMLPrimitives :
  public acpHTMLPage
{
  public:
  			acpGarciaHTMLPrimitives(acpGarciaInternal* pGarcia) : 
  			  acpHTMLPage("agAPIViewPrimitives.tpl"),
  			  m_pcGarcia(pGarcia),
  			  m_pCurObject(NULL),
  			  m_nObjectIndex(-1) {}

  protected:
    aErr		requestCB(const unsigned int nParamIndex,
				  const unsigned int nBlockIndex);

    acpGarciaInternal*	m_pcGarcia;
    acpObject*		m_pCurObject;

    int			m_nObjectIndex;
};


/////////////////////////////////////////////////////////////////////

class acpGarciaHTMLPrimitiveParams :
  public acpHTMLPage
{
  public:
  			acpGarciaHTMLPrimitiveParams(acpGarciaInternal* pGarcia) : 
  			  acpHTMLPage("agAPIViewPrimitiveParams.tpl"),
  			  m_pcGarcia(pGarcia),
  			  m_pcPrimitive(NULL) {}

  protected:
    aErr		requestCB(const unsigned int nParamIndex,
				  const unsigned int nBlockIndex);

    acpGarciaInternal*	m_pcGarcia;
    acpGarciaPrimitive* m_pcPrimitive;
};


/////////////////////////////////////////////////////////////////////

class acpGarciaHTMLAddBehavior :
  public acpHTMLPage
{
  public:
  			acpGarciaHTMLAddBehavior(acpGarciaInternal* pGarcia) : 
  			  acpHTMLPage("agAPIAddBehavior.tpl"),
  			  m_pcGarcia(pGarcia),
  			  m_pcPrimitive(NULL) {}

  protected:
    aErr		requestCB(const unsigned int nParamIndex,
				  const unsigned int nBlockIndex);

    acpGarciaInternal*	m_pcGarcia;
    acpGarciaPrimitive* m_pcPrimitive;
};

#endif // _aGarciaHTMLMain_H_
