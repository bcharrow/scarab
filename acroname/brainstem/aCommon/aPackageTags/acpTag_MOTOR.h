/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_MOTOR.h					           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Package tag class.				   //
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

#ifndef _acpTag_MOTOR_H_
#define _acpTag_MOTOR_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpVec3.h"
#include "acpStringIO.h"
#include "acpFloat.h"


/////////////////////////////////////////////////////////////////////
// MOTOR = 0x000B

class acpTag_MOTOR :
  public acpPackageTag
{
  public:
	  			acpTag_MOTOR();
  				~acpTag_MOTOR();

    void			setData(
				  const aShort nOwnerID,
				  const char* pName,
				  const acpVec3& translation,
    				  const acpVec3& axis,
    				  const aFloat torque,
    				  const aFloat torqueMin);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpStringIO			m_name;
    acpVec3			m_translation;
    acpVec3			m_axis;
    acpFloat			m_torque;
    acpFloat			m_torqueMin;

  friend class			acpMotor;
};

#endif // _acpTag_MOTOR_H_
