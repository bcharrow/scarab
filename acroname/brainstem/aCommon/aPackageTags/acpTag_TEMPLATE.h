/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_TEMPLATE.h				           //
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

#ifndef _acpTag_TEMPLATE_H_
#define _acpTag_TEMPLATE_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpString.h"



/////////////////////////////////////////////////////////////////////
// TEMPLATE = 0x0011

class acpTag_TEMPLATE :
  public acpPackageTag
{
  public:
	  			acpTag_TEMPLATE();
    virtual			~acpTag_TEMPLATE();

    void			setData(
				  const aShort nOwnerID,
				  const char* pName,
				  const aShort nPropCt,
				  const aInt32 nBufferSize);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpString			m_name;
    aShort			m_nPropCt;
    aInt32			m_nBufferSize;

  friend class			acpRobotPackager;
  friend class			acpRobotInternal;
  friend class			acpRobotBaseObject;
};

#endif // _acpTag_TEMPLATE_H_
