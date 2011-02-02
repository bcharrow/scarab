/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaServo.h                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API servo object.         //
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

#ifndef _acpGarciaServo_H_
#define _acpGarciaServo_H_

#include "acpObject.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpGarciaInternal.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the servo class

#define aGSP_POSITION_ABS		"position-abs"
#define aGSP_POSITION_REL		"position-rel"
#define aGSP_ENABLE_FLAG		"enable-flag"
#define aGSP_INVERT_FLAG		"invert-flag"
#define aGSP_DISABLEDSTATE		"disabled-state"
#define aGSP_SPEED			"speed"
#define aGSP_OFFSET			"offset"
#define aGSP_RANGE			"range"

#define	aGSP_SERVO_0			"servo_0"
#define	aGSP_SERVO_1			"servo_1"
#define	aGSP_SERVO_2			"servo_2"
#define	aGSP_SERVO_3			"servo_3"



class acpGarciaInternal;
class acpGarciaServoProperty;

class acpGarciaServo :
  public acpObject
{
  protected:
  				acpGarciaServo(
				  const char* pName,
  				  const char cModule,
  				  const char cIndex,
  				  acpGarciaInternal* pcGarciaInternal);

  public:
    virtual   			~acpGarciaServo();

    virtual aErr		writeToStream(
    				  const aStreamRef stream) const;

    virtual void		getDescription(acpTextBuffer& buffer) {}

  protected:

    char			m_cModule;
    char			m_cIndex;
    acpGarciaInternal*		m_pcGarciaInternal;

  friend class acpGarciaInternal;
  friend class acpGarciaServoProperty;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaServoProperty : public acpProperty
{
  public:
  				acpGarciaServoProperty(
  				  acpGarciaServo* pcGarciaServo,
  				  const char* pName,
  				  aPROPERTY_FLAGS flags);

    virtual			~acpGarciaServoProperty() {}

  protected:

    void			sendServoPacket (
		  		  acpGarciaServo* pcGarciaServo,
				  char cCmd,
				  char data1);

    void			sendServoPacket (
	  			  acpGarciaServo* pcGarciaServo,
				  char cCmd,
				  char data1,
				  char data2);

    float			getServoPositionValue() const
				  { return m_pcGarciaServo->m_pcGarciaInternal->getServoPositionValue(
				      (unsigned char)m_pcGarciaServo->m_cModule, (unsigned char)m_pcGarciaServo->m_cIndex); }

    unsigned char		getServoConfigValue() const
				  { return m_pcGarciaServo->m_pcGarciaInternal->getServoConfigValue(
				      (unsigned char)m_pcGarciaServo->m_cModule, (unsigned char)m_pcGarciaServo->m_cIndex); }

    short			getServoLimitsValue() const
				  { return m_pcGarciaServo->m_pcGarciaInternal->getServoLimitsValue(
				      (unsigned char)m_pcGarciaServo->m_cModule, (unsigned char)m_pcGarciaServo->m_cIndex); }

  protected:

    acpGarciaServo*		m_pcGarciaServo;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaServoPosAbsProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoPosAbsProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServoPosRelProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoPosRelProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServoEnableProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoEnableProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServoInvertProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoInvertProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServoDisabledStateProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoDisabledStateProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServoSpeedProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoSpeedProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServoOffsetProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoOffsetProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServoRangeProperty : public acpGarciaServoProperty
{
  public:
  				acpGarciaServoRangeProperty(
  				  acpGarciaServo* pcGarciaServo);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};

#endif // _acpGarciaServo_H_
