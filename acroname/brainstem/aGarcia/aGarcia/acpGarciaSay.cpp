/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaSay.cpp                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API primitive object. //
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

#include "aCmd.tea"

#include "aUtil.h"
#include "aStem.h"
#include "acpGarciaInternal.h"
#include "acpException.h"
#include "acpHTMLPage.h"
#include "acpBehavior.h"
#include "acpGarciaSay.h"
#include "acpPacketMessage.h"
#include "aSP03.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the say behavior

#define aGPP_SAY_PRIMITIVE_NAME		"say"

#define aGPP_SAY_PHRASE			"phrase"
#define aGPP_SAY_SPEED			"speed"
#define aGPP_SAY_PITCH			"pitch"
#define aGPP_SAY_VOLUME			"volume"



/////////////////////////////////////////////////////////////////////
// acpGarciaSay constructor
//

acpGarciaSay::acpGarciaSay (
  acpGarciaInternal* pGarcia
) :
  acpGarciaPrimitive(aGPP_SAY_PRIMITIVE_NAME, pGarcia)
{
  m_nCodeSize = 0;
  m_nModule = 2;

  acpValue defPhrase("");
  m_nPhrasePropIndex =
    addProperty(new acpSayBehaviorPhraseProperty(this), &defPhrase);

  acpValue defSpeed(0.7f);
  m_nSpeedPropIndex =
    addProperty(new acpSayBehaviorSpeedProperty(this), &defSpeed);

  acpValue defPitch(0.6f);
  m_nPitchPropIndex =
    addProperty(new acpSayBehaviorPitchProperty(this), &defPitch);

  acpValue defVolume(1.0f);
  m_nVolumePropIndex =
    addProperty(new acpSayBehaviorVolumeProperty(this), &defVolume);

} // acpGarciaSay constructor



/////////////////////////////////////////////////////////////////////
// acpGarciaSay execute method
//

void acpGarciaSay::execute(acpBehavior* pBehavior)
{
  // get the params
  int q;
  const char* pString = 
    pBehavior->getValue(m_nPhrasePropIndex)->getStringVal();
  q=(int)aStringLen(pString);
  float fSpeed = 
    pBehavior->getValue(m_nSpeedPropIndex)->getFloatVal();
  float fPitch = 
    pBehavior->getValue(m_nPitchPropIndex)->getFloatVal();
  float fVolume = 
    pBehavior->getValue(m_nVolumePropIndex)->getFloatVal();
  
  if (aStringLen(pString)) {
    
    unsigned char nSpeed = (unsigned char)(fSpeed * 3);
    unsigned char nPitch = (unsigned char)(fPitch * 7);
    unsigned char nVolume = (unsigned char)((1.0f - fVolume) * 7);
    aSP03_SpeakString(m_pcGarciaInternal->m_stemRef, 
    		      nVolume, nPitch, nSpeed, pString);
  }

} // acpGarciaSay execute method



/////////////////////////////////////////////////////////////////////
// acpGarciaSay getParamHTML method
//

const char* acpGarciaSay::getParamHTML()
{
  return "Phrase: <INPUT NAME='phrase' VALUE='' SIZE='20'><BR>"
  	 "Speed: <INPUT NAME='speed' VALUE='0.7' SIZE='3'><BR>"
  	 "Pitch: <INPUT NAME='pitch' VALUE='0.6' SIZE='3'><BR>"
  	 "Volume: <INPUT NAME='volume' VALUE='1.0' SIZE='3'>";

} // acpGarciaSay getParamHTML method



/////////////////////////////////////////////////////////////////////
// phrase property
/////////////////////////////////////////////////////////////////////

acpSayBehaviorPhraseProperty::acpSayBehaviorPhraseProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_SAY_PHRASE,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_STRING))
{
}



/////////////////////////////////////////////////////////////////////
// speed property
/////////////////////////////////////////////////////////////////////

acpSayBehaviorSpeedProperty::acpSayBehaviorSpeedProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_SAY_SPEED,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
// pitch property
/////////////////////////////////////////////////////////////////////

acpSayBehaviorPitchProperty::acpSayBehaviorPitchProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_SAY_PITCH,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
// volume property
/////////////////////////////////////////////////////////////////////

acpSayBehaviorVolumeProperty::acpSayBehaviorVolumeProperty (
  				  acpGarciaPrimitive* pPrimitive) :
  acpBehaviorProperty(pPrimitive,
  		      aGPP_SAY_VOLUME,
  		      (aPROPERTY_FLAGS)(aPROPERTY_FLAG_USERBIT
  		      		      | aPROPERTY_FLAG_WRITE
  		      		      | aPROPERTY_FLAG_READ
  		      		      | aPROPERTY_FLAG_FLOAT))
{
}
