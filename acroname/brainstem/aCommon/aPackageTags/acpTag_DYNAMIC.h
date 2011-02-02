/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_DYNAMIC.h				           //
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

#ifndef _acpTag_DYNAMIC_H_
#define _acpTag_DYNAMIC_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpFloat.h"
#include "acpStringIO.h"
#include "acpVec3.h"
#include "acpMatrix3.h"



/////////////////////////////////////////////////////////////////////
// DYNAMIC = 0x0005

class acpTag_DYNAMIC :
  public acpPackageTag
{
  public:
  				acpTag_DYNAMIC() :
  				  acpPackageTag(aTAG_DYNAMIC, (void*)true),
  				  m_mass(0.0f),
				  m_friction(0.0f),
				  m_soft_erp(0.0f),
				  m_soft_cfm(0.0f),
				  m_bounce(0.0f)
  				  {}
  				~acpTag_DYNAMIC();

    void			setData(
      				  const aShort nOwnerID,
      				  const char* pName,
      				  const float fMass,
				  const float fFriction,
				  const float fSoftERP,
				  const float fSoftCFM,
				  const float fBounce,
      				  const acpVec3& position,
      				  const acpMatrix3& rotation);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpStringIO			m_name;
    acpFloat			m_mass;
    acpFloat			m_friction;
    acpFloat			m_soft_erp;
    acpFloat			m_soft_cfm;
    acpFloat			m_bounce;
    acpVec3			m_position;
    acpMatrix3			m_rotation;

  friend class			acpDynamic;
};

#endif // _acpTag_DYNAMIC_H_
