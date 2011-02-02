/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpByte.h 		 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of basic byte class.                    //
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

#ifndef _acpByte_H_
#define _acpByte_H_

#include "aIO.h"

class acpByte
{
  public:
                                acpByte() :
				  m_val(0)
				  {}
                                acpByte(
				  acpByte& b) :
				  m_val(b.m_val)
				  {}
                                acpByte(
				  const acpByte& b) :
				  m_val(b.m_val)
				  {}
                                acpByte(
				  const int i) :
				  m_val((aByte)i)
				  {}
				acpByte(
				  aStreamRef stream);
				acpByte(
				  const aByte c) :
				  m_val(c)
				  {}

    aErr			writeToStream(
				  aStreamRef stream) const;

    				operator aByte() const
    				  { return m_val; }

  private:
    aByte			m_val;
};

#endif // _acpByte_H_
