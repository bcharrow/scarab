/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaSleep.h                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API sleep primitive.      //
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


#ifndef _acpGarciaSleep_H_
#define _acpGarciaSleep_H_

#include "acpGarciaPrimitive.h"

class acpGarciaSleep :
  public acpGarciaPrimitive
{
  public:
	  		acpGarciaSleep(acpGarciaInternal* pGarcia);
  			~acpGarciaSleep() {}

    void		execute(acpBehavior* pBehavior);
    
    const char*		getParamHTML();

  private:
    int			m_nDurationPropIndex;
};


/////////////////////////////////////////////////////////////////////

class acpSleepBehaviorDurationProperty : public acpBehaviorProperty
{
  public:
  				acpSleepBehaviorDurationProperty(
  				  acpGarciaPrimitive* pPrimitive);
};

#endif // _acpGarciaSleep_H_
