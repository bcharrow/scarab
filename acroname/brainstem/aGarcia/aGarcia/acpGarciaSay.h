/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaSay.h                                            //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API say primitive.        //
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
// arising out of or in connection with this software,fo even if     //
// Acroname Inc. has been advised of the possibility of such       //
// damages.                                                        //
//                                                                 //
// Acroname Inc.                                                   //
// www.acroname.com                                                //
// 720-564-0373                                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////


#ifndef _acpGarciaSay_H_
#define _acpGarciaSay_H_

#include "acpGarciaPrimitive.h"

class acpGarciaSay :
  public acpGarciaPrimitive
{
  public:
	  		acpGarciaSay(acpGarciaInternal* pGarcia);
  			~acpGarciaSay() {}

    void		execute(acpBehavior* pBehavior);
    
    const char*		getParamHTML();

  private:
    int			m_nPhrasePropIndex;
    int			m_nSpeedPropIndex;
    int			m_nPitchPropIndex;
    int			m_nVolumePropIndex;
};


/////////////////////////////////////////////////////////////////////

class acpSayBehaviorPhraseProperty : public acpBehaviorProperty
{
  public:
  				acpSayBehaviorPhraseProperty(
  				  acpGarciaPrimitive* pPrimitive);
};

/////////////////////////////////////////////////////////////////////

class acpSayBehaviorSpeedProperty : public acpBehaviorProperty
{
  public:
  				acpSayBehaviorSpeedProperty(
  				  acpGarciaPrimitive* pPrimitive);
};

/////////////////////////////////////////////////////////////////////

class acpSayBehaviorPitchProperty : public acpBehaviorProperty
{
  public:
  				acpSayBehaviorPitchProperty(
  				  acpGarciaPrimitive* pPrimitive);
};

/////////////////////////////////////////////////////////////////////

class acpSayBehaviorVolumeProperty : public acpBehaviorProperty
{
  public:
  				acpSayBehaviorVolumeProperty(
  				  acpGarciaPrimitive* pPrimitive);
};


#endif // _acpGarciaSay_H_
