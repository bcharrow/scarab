/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_USEROBJECT.h				           //
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

#ifndef _acpTag_USEROBJECT_H_
#define _acpTag_USEROBJECT_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpString.h"



/////////////////////////////////////////////////////////////////////
// USEROBJECT = 0x0012

class acpTag_USEROBJECT :
  public acpPackageTag
{
  public:
	  			acpTag_USEROBJECT();
    virtual			~acpTag_USEROBJECT();

    void			setData(
				  const aShort nOwnerID,
				  const char* pName,
				  const char* pLinkName,
				  const char* pTemplateName,
				  const int nInitCt);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpString			m_name;
    acpString			m_linkname;
    acpString			m_templatename;
    aInt32			m_nInitCt;

  friend class			acpRobotPackager;
  friend class			acpRobotInternal;
  friend class			acpRobotUserObject;
};

#endif // _acpTag_USEROBJECT_H_
