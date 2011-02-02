/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_INITIALIZER.h				           //
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

#ifndef _acpTag_INITIALIZER_H_
#define _acpTag_INITIALIZER_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpStringIO.h"



/////////////////////////////////////////////////////////////////////
// INITIALIZER = 0x0013

class acpTag_INITIALIZER :
  public acpPackageTag
{
  public:
	  			acpTag_INITIALIZER();
    virtual			~acpTag_INITIALIZER();

    void			setData(
				  const aShort nOwnerID,
				  const char* pName,
				  const char* pSvalue);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpString			m_name;
    acpString			m_svalue;

  friend class			acpRobotPackager;
  friend class			acpRobotInternal;
  friend class			acpRobotUserObject;
};

#endif // _acpTag_INITIALIZER_H_
