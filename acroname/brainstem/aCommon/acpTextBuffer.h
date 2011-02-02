/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTextBuffer.h                                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of a text buffer object.                //
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

#ifndef _acpTextBuffer_H_
#define _acpTextBuffer_H_

#include "aIO.h"

class acpTextBuffer 
{
  public:
 			acpTextBuffer(aIOLib ioRef);
 			~acpTextBuffer();

    void		add(const char* pText);
    void		add(const float fValue);
    void		add(const int nValue);
 
    const char*		getBuffer();

  private:
    aIOLib		m_ioRef;
    aStreamRef		m_buffer;
};

#endif // _acpTextBuffer_H_
