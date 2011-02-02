/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_VIEWS.h		 	 		           //
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

#ifndef _acpTag_VIEWS_H_
#define _acpTag_VIEWS_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"



/////////////////////////////////////////////////////////////////////
// VIEWS = 0x0001

class acpTag_VIEWS :
  public acpPackageTag
{
  public:
  			acpTag_VIEWS() :
  			  acpPackageTag(aTAG_VIEWS, (void*)false),
  			  m_nDataSize(0),
  			  m_pData(NULL)
  			  {}
  			~acpTag_VIEWS();

    void		setData(
    			  const aShort nSize,
    			  const aByte* pData);
    
    acpPackageTag*	factoryFromStream(
    			  aStreamRef stream);

  protected:
    void		writeData(
    			  aStreamRef stream) const; 

  private:
    aShort		m_nDataSize;
    aByte*		m_pData;
    
  friend class		acpSymonym;
};

#endif // _acpTag_VIEWS_H_
