/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGetSonarMessage.h                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the sonar messages sent to the       //
//              private thread in the Garcia API library object.   //
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

#ifndef _acpGetSonarMessage_H_
#define _acpGetSonarMessage_H_

#include "acpGarciaInternal.h"

class acpGetSonarRangeMessage :
  public acpMessage
{
  public:

  			acpGetSonarRangeMessage(
  			  acpGarciaInternal* pGarcia,
  			  const unsigned char module,
  			  const unsigned char addr,
  			  const unsigned char units,
  			  const int nvals,
  			  short* psBuff,
  			  int* pnValue) :
  			    m_module(module),
  			    m_addr(addr),
  			    m_units(units),
  			    m_nvals(nvals),
  			    m_psBuff(psBuff),
  			    m_pnValue(pnValue),
  			    m_pcGarcia(pGarcia) {}

    void		process();

    unsigned char	m_module;
    unsigned char	m_addr;
    unsigned char	m_units;
    int			m_nvals;
    short*		m_psBuff;
    int*		m_pnValue;

  private:

    acpGarciaInternal*	m_pcGarcia;
};

class acpGetSonarLightMessage :
  public acpMessage
{
  public:

  			acpGetSonarLightMessage(
  			  acpGarciaInternal* pGarcia,
  			  const unsigned char module,
  			  const unsigned char addr,
  			  int* pnValue) :
  			    m_module(module),
  			    m_addr(addr),
  			    m_pnValue(pnValue),
  			    m_pcGarcia(pGarcia) {}

    void		process();

    unsigned char	m_module;
    unsigned char	m_addr;
    int*		m_pnValue;

  private:

    acpGarciaInternal*	m_pcGarcia;
};

#endif // _acpGetSonarMessage_H_
