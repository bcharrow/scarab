/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_MOTOR.h 		 		           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of XML tag class.                       //
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

#ifndef _acpXmlTag_MOTOR_H_
#define _acpXmlTag_MOTOR_H_

#include "acpPkgXML.h"

#include "acpList.h"
#include "acpXmlTag_NAME.h"
#include "acpXmlTag_AXIS.h"
#include "acpXmlTag_TRANSLATION.h"
#include "acpXmlTag_DYNAMIC.h"
#include "acpXmlTag_MOTORLINK.h"
#include "acpXmlTag_TORQUE.h"
#include "acpXmlTag_TORQUEMIN.h"


class MOTOR :
  public acpPkgXML
{
  public:
  			MOTOR(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			  acpPkgXML(pImporter, pParent),
			  m_pName(NULL),
			  m_pAxis(NULL),
			  m_pTranslation(NULL),
			  m_pMotorLink(NULL),
			  m_pTorque(NULL),
			  m_pTorqueMin(NULL)
  			  {}
  			~MOTOR();

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new MOTOR(m_pImporter, 
					     pParent); }

    bool		addTag(
    			  acpPkgXML* pTag);

    const char*  	tagName() const
  			  { return "MOTOR"; }

    char*		getObjectName();

    void		traverse();

  private:
    NAME*		m_pName;
    AXIS*		m_pAxis;
    TRANSLATION*        m_pTranslation;
    acpList<DYNAMIC>	m_dynamics;
    MOTORLINK*		m_pMotorLink;
    TORQUE*		m_pTorque;
    TORQUEMIN*		m_pTorqueMin;
};

#endif // _acpXmlTag_MOTOR_H_
