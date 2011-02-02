/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaProperties.h                                     //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definitions of the Garcia API library object.      //
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

#ifndef _acpGarciaProperties_H_
#define _acpGarciaProperties_H_

#include "acpGarciaInternal.h"
#include "acpProperty.h"


/////////////////////////////////////////////////////////////////////

class acpGarciaProperty : 
  public acpProperty
{
  public:
  		acpGarciaProperty(
  		  acpGarciaInternal* pGarciaInternal,
  		  const char* pName,
  		  aPROPERTY_FLAGS flags);
  protected:
    void	sendStemPacket (
  		  unsigned char module,
		  unsigned char length,
		  char* data)
		  { m_pGarciaInternal->sendStemPacket (
		      module, length, data); }
    char	getPadValue (
  		  unsigned char module,
		  unsigned char offset) const
		  { return m_pGarciaInternal->getPadValue(
				      module, offset); }
    void	getRangerValue (
  		  unsigned char id,
		  unsigned char code,
		  acpValue* pValue) const
		  { m_pGarciaInternal->getRangerValue(
				    id, code, pValue); }

    float	getAnalogValue (
  		  unsigned char module,
		  unsigned char analogID) const
		  { return m_pGarciaInternal->getAnalogValue(
				    module, analogID); }

    float	getDigitalValue (
  		  unsigned char module,
		  unsigned char digitalID) const
		  { return (float)(m_pGarciaInternal->getDigitalValue(
				    module, digitalID)); }

    float	getOdometerValue (
  		  unsigned char module,
		  unsigned char motorID) const
		  { return m_pGarciaInternal->getOdometerValue(
				    module, motorID); }

    short	getCounterValue (
  		  unsigned char module,
		  unsigned char counterID) const
		  { return m_pGarciaInternal->getCounterValue(
				    module, counterID); }

    acpGarciaInternal*		m_pGarciaInternal;
};


/////////////////////////////////////////////////////////////////////

class acpGarciaActiveProperty : public acpGarciaProperty
{
  public:
  		acpGarciaActiveProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaIdleProperty : public acpGarciaProperty
{
  public:
  		acpGarciaIdleProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaUserLEDProperty : public acpGarciaProperty
{
  public:
  		acpGarciaUserLEDProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaUserButtonProperty : public acpGarciaProperty
{
  public:
  		acpGarciaUserButtonProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaBatteryLevelProperty : public acpGarciaProperty
{
  public:
  		acpGarciaBatteryLevelProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaBatteryVoltageProperty : public acpGarciaProperty
{
  public:
  		acpGarciaBatteryVoltageProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaDampedSpeedLeftProperty : public acpGarciaProperty
{
  public:
  		acpGarciaDampedSpeedLeftProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
		  const acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaDampedSpeedRightProperty : public acpGarciaProperty
{
  public:
  		acpGarciaDampedSpeedRightProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaHeartbeatCallbackProperty : public acpGarciaProperty
{
  public:
  		acpGarciaHeartbeatCallbackProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaHeartbeatStatusProperty : public acpGarciaProperty
{
  public:
  		acpGarciaHeartbeatStatusProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaRangerProperty : public acpGarciaProperty
{
  public:
  		acpGarciaRangerProperty (
  		  acpGarciaInternal* m_pGarciaInternal,
  		  const char* pName,
  		  unsigned char cID,
  		  unsigned char cCode,
		  aPROPERTY_FLAGS flags);

    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
  protected:
    unsigned char	m_cID;
    unsigned char	m_cCode;
};


/////////////////////////////////////////////////////////////////////

class acpGarciaDownRangerLeftProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaDownRangerLeftProperty (
  		  acpGarciaInternal* m_pGarciaInternal);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaDownRangerRightProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaDownRangerRightProperty (
  		  acpGarciaInternal* m_pGarciaInternal);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaFrontRangerLeftProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaFrontRangerLeftProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaFrontRangerRightProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaFrontRangerRightProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaSideRangerLeftProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaSideRangerLeftProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaSideRangerRightProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaSideRangerRightProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaRearRangerLeftProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaRearRangerLeftProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaRearRangerRightProperty : public acpGarciaRangerProperty
{
  public:
  		acpGarciaRearRangerRightProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};


/////////////////////////////////////////////////////////////////////

class acpGarciaRangerThresholdProperty : public acpGarciaProperty
{
  public:
  		acpGarciaRangerThresholdProperty (
  		  acpGarciaInternal* m_pGarciaInternal,
  		  const char* pName,
  		  eGarciaRangerType rangerType,
  		  unsigned char cModule,
  		  unsigned char cPadIndex);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* value);
  protected:
    eGarciaRangerType	m_rangerType;
    unsigned char	m_cModule;
    unsigned char	m_cPadIndex;
};

/////////////////////////////////////////////////////////////////////

class acpGarciaFrontRangerThresholdProperty : public acpGarciaRangerThresholdProperty
{
  public:
  		acpGarciaFrontRangerThresholdProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaSideRangerThresholdProperty : public acpGarciaRangerThresholdProperty
{
  public:
  		acpGarciaSideRangerThresholdProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaRearRangerThresholdProperty : public acpGarciaRangerThresholdProperty
{
  public:
  		acpGarciaRearRangerThresholdProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};


/////////////////////////////////////////////////////////////////////

class acpGarciaRangerEnableProperty : public acpGarciaProperty
{
  public:
  		acpGarciaRangerEnableProperty (
  		  acpGarciaInternal* m_pGarciaInternal,
  		  const char* pName,
  		  char cBit);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
  protected:
    char	m_cBit;
};

/////////////////////////////////////////////////////////////////////

class acpGarciaDownRangerEnableProperty : public acpGarciaRangerEnableProperty
{
  public:
  		acpGarciaDownRangerEnableProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaFrontRangerEnableProperty : public acpGarciaRangerEnableProperty
{
  public:
  		acpGarciaFrontRangerEnableProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaSideRangerEnableProperty : public acpGarciaRangerEnableProperty
{
  public:
  		acpGarciaSideRangerEnableProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};

/////////////////////////////////////////////////////////////////////

class acpGarciaRearRangerEnableProperty : public acpGarciaRangerEnableProperty
{
  public:
  		acpGarciaRearRangerEnableProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

};


/////////////////////////////////////////////////////////////////////

class acpGarciaDistanceUnitsProperty : public acpGarciaProperty
{
  public:
  		acpGarciaDistanceUnitsProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaDistanceUnitsStringProperty : public acpGarciaProperty
{
  public:
  		acpGarciaDistanceUnitsStringProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaAngleUnitsProperty : public acpGarciaProperty
{
  public:
  		acpGarciaAngleUnitsProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaAngleUnitsStringProperty : public acpGarciaProperty
{
  public:
  		acpGarciaAngleUnitsStringProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaIRTransmitProperty : public acpGarciaProperty
{
  public:
  		acpGarciaIRTransmitProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaIRReceiveProperty : public acpGarciaProperty
{
  public:
  		acpGarciaIRReceiveProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaUserFlagsProperty : public acpGarciaProperty
{
  public:
  		acpGarciaUserFlagsProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaStatusProperty : public acpGarciaProperty
{
  public:
  		acpGarciaStatusProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaDistanceLeftProperty : public acpGarciaProperty
{
  public:
  		acpGarciaDistanceLeftProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaDistanceRightProperty : public acpGarciaProperty
{
  public:
  		acpGarciaDistanceRightProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaStallThresholdProperty : public acpGarciaProperty
{
  public:
  		acpGarciaStallThresholdProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaStallQueueSizeProperty : public acpGarciaProperty
{
  public:
  		acpGarciaStallQueueSizeProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaSpeedProperty : public acpGarciaProperty
{
  public:
  		acpGarciaSpeedProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaStatusStreamProperty : public acpGarciaProperty
{
  public:
  		acpGarciaStatusStreamProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaErrorStreamProperty : public acpGarciaProperty
{
  public:
  		acpGarciaErrorStreamProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaRelayIndexProperty : public acpGarciaProperty
{
  public:
  		acpGarciaRelayIndexProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaRelayStatusProperty : public acpGarciaProperty
{
  public:
  		acpGarciaRelayStatusProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaRelayByteProperty : public acpGarciaProperty
{
  public:
  		acpGarciaRelayByteProperty (
  		  acpGarciaInternal* m_pGarciaInternal);

    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServo0Property : public acpGarciaProperty
{
  public:
  		acpGarciaServo0Property (
  		  acpGarciaInternal* m_pGarciaInternal);
};

/////////////////////////////////////////////////////////////////////

class acpGarciaServo1Property : public acpGarciaProperty
{
  public:
  		acpGarciaServo1Property (
  		  acpGarciaInternal* m_pGarciaInternal);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServo2Property : public acpGarciaProperty
{
  public:
  		acpGarciaServo2Property (
  		  acpGarciaInternal* m_pGarciaInternal);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaServo3Property : public acpGarciaProperty
{
  public:
  		acpGarciaServo3Property (
  		  acpGarciaInternal* m_pGarciaInternal);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaCameraBoomProperty : public acpGarciaProperty
{
  public:
  		acpGarciaCameraBoomProperty (
  		  acpGarciaInternal* m_pGarciaInternal);
};

#endif // _acpGarciaProperties_H_

