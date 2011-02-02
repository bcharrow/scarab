/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_INSTANCE.h	 	 		           //
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

#ifndef _acpTag_INSTANCE_H_
#define _acpTag_INSTANCE_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpTransform.h"
#include "acpString.h"
#include "acpShort.h"


// flag fields
#define kInstance_Visible	0x01
#define kInstance_Static	0x02



/////////////////////////////////////////////////////////////////////
// INSTANCE = 0x000C

class acpTag_INSTANCE :
  public acpPackageTag
{
  public:
	  			acpTag_INSTANCE();
  				~acpTag_INSTANCE();

    void			setData(
				  const aShort nOwnerID,
				  const char* pName,
				  const acpList<acpShort>& geometry,
				  const aByte nFlags,
				  const acpTransform& transform);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

    const acpList<acpShort>&	getGeometryIDs() const
    				    { return m_geometryIDs; }

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpString			m_name;
    acpList<acpShort>		m_geometryIDs;
    aByte			m_nFlags;
    acpVec3			m_translation;
    acpMatrix3			m_rotation;

  friend class			acpInstance;
};

#endif // _acpTag_INSTANCE_H_
