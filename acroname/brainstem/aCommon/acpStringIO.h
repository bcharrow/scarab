/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpStringIO.h                                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the acpString IO utility object.     //
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

#ifndef _acpStringIO_H_
#define _acpStringIO_H_

#include "aIO.h"
#include "acpString.h"

class acpStringIO : public acpString {
  public:
			acpStringIO() :
			  acpString() {}
			acpStringIO(const acpString& string) :
			  acpString(string) {}
                        acpStringIO(const aStreamRef stream);
			acpStringIO(const char* pValue) :
			  acpString(pValue) {}
    virtual             ~acpStringIO() {}

    aErr		writeToStream(aStreamRef stream) const;
  
    aStreamRef          getStream(aIOLib ioRef);

  private:
    static aErr         sStreamGet(char* pData,
			          void* ref);
    size_t		m_current;
};

#endif // _acpStringIO_H_
