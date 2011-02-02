/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaSonar.h                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API sonar object.         //
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

#ifndef _acpGarciaSonar_H_
#define _acpGarciaSonar_H_

#include "acpObject.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpGarciaInternal.h"
#include "aSRF08Defs.tea"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the sonar object

#define aGRP_ADDRESS			"address"
#define aGRP_RANGE_INCH			"range"
#define aGRP_LIGHT_VALUE		"light-value"
#define aGRP_RANGE_BUFFER		"range-buffer"
#define aGRP_ECHO_COUNT			"echo-count"

#define aGRP_SONAR_PREFIX		"sonar_"
#define	aGRP_SONAR_0			"sonar_0"
#define	aGRP_SONAR_1			"sonar_1"
#define	aGRP_SONAR_2			"sonar_2"
#define	aGRP_SONAR_3			"sonar_3"
#define	aGRP_SONAR_4			"sonar_4"
#define	aGRP_SONAR_5			"sonar_5"
#define	aGRP_SONAR_6			"sonar_6"
#define	aGRP_SONAR_7			"sonar_7"
#define	aGRP_SONAR_8			"sonar_8"
#define	aGRP_SONAR_9			"sonar_9"
#define	aGRP_SONAR_10			"sonar_10"
#define	aGRP_SONAR_11			"sonar_11"
#define	aGRP_SONAR_12			"sonar_12"
#define	aGRP_SONAR_13			"sonar_13"
#define	aGRP_SONAR_14			"sonar_14"
#define	aGRP_SONAR_15			"sonar_15"

#define	aGRP_VSOUND_MPERS		346.65f
#define	aGRP_VSOUND_FPERS		1137.303f
#define	aGRP_VSOUND_IPERS		94.775f



class acpGarciaInternal;
class acpGarciaSonarProperty;

class acpGarciaSonar :
  public acpObject
{
  protected:
  				acpGarciaSonar(
				  const char* pName,
  				  const unsigned char cModule,
  				  const unsigned char ucAddr,
  				  acpGarciaInternal* pcGarciaInternal);

  public:
    virtual   			~acpGarciaSonar();

    virtual aErr		writeToStream(
    				  const aStreamRef stream) const;

    virtual void		getDescription(acpTextBuffer& buffer) {}

  protected:

    unsigned char		m_cModule;
    unsigned char		m_cSRF08addr;
    int				m_nEchoCount;
    short			m_pRangeBuff[aSRF08_NMAXREADINGS];
    
    acpGarciaInternal*		m_pcGarciaInternal;

  friend class acpGarciaInternal;
  friend class acpGarciaSonarProperty;
  friend class acpGarciaSonarAddressProperty;
  friend class acpGarciaSonarRangeProperty;
  friend class acpGarciaSonarRangeBufferProperty;
  friend class acpGarciaSonarLightValueProperty;
  friend class acpGarciaSonarEchoCountProperty;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaSonarProperty : public acpProperty
{
  public:
  				acpGarciaSonarProperty(
  				  acpGarciaSonar* pcGarciaSonar,
  				  const char* pName,
  				  aPROPERTY_FLAGS flags);

    virtual			~acpGarciaSonarProperty() {}

  protected:

    int				getDistanceUnitType() const;

    int				getSonarRangeValue(
				  unsigned char units);

    int				getSonarLightValue() const;

  protected:

    acpGarciaSonar*		m_pcGarciaSonar;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaSonarAddressProperty : public acpGarciaSonarProperty
{
  public:
  				acpGarciaSonarAddressProperty(
  				  acpGarciaSonar* pcGarciaSonar);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaSonarRangeProperty : public acpGarciaSonarProperty
{
  public:
  				acpGarciaSonarRangeProperty(
  				  acpGarciaSonar* pcGarciaSonar);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaSonarLightValueProperty : public acpGarciaSonarProperty
{
  public:
  				acpGarciaSonarLightValueProperty(
  				  acpGarciaSonar* pcGarciaSonar);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaSonarRangeBufferProperty : public acpGarciaSonarProperty
{
  public:
  				acpGarciaSonarRangeBufferProperty(
  				  acpGarciaSonar* pcGarciaSonar);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaSonarEchoCountProperty : public acpGarciaSonarProperty
{
  public:
  				acpGarciaSonarEchoCountProperty(
  				  acpGarciaSonar* pcGarciaSonar);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};

#endif // _acpGarciaSonar_H_
