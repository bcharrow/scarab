/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_PRIMITIVE.h				           //
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

#ifndef _acpTag_PRIMITIVE_H_
#define _acpTag_PRIMITIVE_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpString.h"



/////////////////////////////////////////////////////////////////////
// PRIMITIVE = 0x0010

class acpTag_PRIMITIVE :
  public acpPackageTag
{
  public:
	  			acpTag_PRIMITIVE();
    virtual			~acpTag_PRIMITIVE();

    void			setData(
				  const aShort nOwnerID,
				  const char* pName,
				  const char* pLinkName,
				  const char* pApiName,
				  const char* pStemName,
				  const aShort nPropCt);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpString			m_name;
    acpString			m_linkname;
    acpString			m_apifile;
    char*			m_pApiCup;
    unsigned long		m_ulApiCupSize;
    acpString			m_stemfile;
    char*			m_pStemCup;
    unsigned long		m_ulStemCupSize;
    aShort			m_nPropCt;

  friend class			acpRobotPackager;
  friend class			acpRobotInternal;
  friend class			acpRobotUserProperty;
  friend class			acpRobotPrimitive;
};

#endif // _acpTag_PRIMITIVE_H_
