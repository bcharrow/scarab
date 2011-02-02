/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpInt32.h 		 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of basic int (4-byte) class.            //
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

#ifndef _acpInt32_H_
#define _acpInt32_H_

#include "aIO.h"

class acpInt32 
{
  public:
  				acpInt32(
  				  const aInt32 nVal) :
  				  m_val(nVal)
  				  {}
  				acpInt32(
  				  aStreamRef stream);

    void			writeToStream(
    				  aStreamRef stream) const; 

    				operator aInt32() const
    				  { return m_val; }
  private:
    aInt32			m_val;
};

#endif // _acpInt32_H_
