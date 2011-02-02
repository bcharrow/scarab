/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaProperties.cpp                                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Garcia API library object.   //
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
#include "aGarciaDefs.tea"

#include "aUtil.h"
#include "aGarciaGeom.h"
#include "aGarciaProperties.h"

#include "acpGarciaProperties.h"


/////////////////////////////////////////////////////////////////////

acpGarciaProperty::acpGarciaProperty (
  acpGarciaInternal* pGarciaInternal,
  const char* pName,
  aPROPERTY_FLAGS flags
) :
  acpProperty(pName, flags),
  m_pGarciaInternal(pGarciaInternal)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// active
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaActiveProperty::acpGarciaActiveProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_ACTIVE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_BOOL))
{
}
	

/////////////////////////////////////////////////////////////////////

void acpGarciaActiveProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{ 
  pValue->set(m_pGarciaInternal->isActive());
}




/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// idle
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaIdleProperty::acpGarciaIdleProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_IDLE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_BOOL))
{
}
	

/////////////////////////////////////////////////////////////////////

void acpGarciaIdleProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{ 
  pValue->set(m_pGarciaInternal->isIdle());
}




/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// user-led
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaUserLEDProperty::acpGarciaUserLEDProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_LED, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_INT))
{
}
	

/////////////////////////////////////////////////////////////////////

void acpGarciaUserLEDProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  int nValue = pValue->getIntVal();

  data[0] = cmdRAW_INPUT;
  data[1] = aGARCIA_MOTO_RFLX_LED;
  data[2] = (nValue != 0);
  sendStemPacket(aGARCIA_MOTO_ADDR, 3, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaUserLEDProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  int n = 
  getPadValue(
    aGARCIA_MOTO_ADDR, 
    aGARCIA_MOTO_PADB_MIRRORIO);
  if (n) n = 1; // force to be 1 or 0
  pValue->set(n);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// user-button
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaUserButtonProperty::acpGarciaUserButtonProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_BUTTON, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_INT))
{
}
	
/////////////////////////////////////////////////////////////////////

void acpGarciaUserButtonProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  int n = 
  getPadValue(
    aGARCIA_GP_ADDR, 
    aGARCIA_GP_PADB_MIRRORBUTTON);
  if (n) n = 1; // force to be 1 or 0
  pValue->set(n);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// battery-level
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaBatteryLevelProperty::acpGarciaBatteryLevelProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_BATTERYLEVEL, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_FLOAT))
{
}
	
/////////////////////////////////////////////////////////////////////

void acpGarciaBatteryLevelProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(aGarciaGeom_VoltageToCapacity(
                getAnalogValue(aGARCIA_GP_ADDR,
                               aGARCIA_GP_ABATTERY) * 2.0f));
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// battery-voltage
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaBatteryVoltageProperty::acpGarciaBatteryVoltageProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_BATTERYVOLTAGE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_FLOAT))
{
}
	
/////////////////////////////////////////////////////////////////////

void acpGarciaBatteryVoltageProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(getAnalogValue(aGARCIA_GP_ADDR,
                           aGARCIA_GP_ABATTERY) * 2.0f);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// damped-speed-left
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDampedSpeedLeftProperty::acpGarciaDampedSpeedLeftProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_DAMPEDSPEEDL, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_FLOAT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDampedSpeedLeftProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  float fValue = pValue->getFloatVal();
  short motoSpeed = aGarciaGeom_SpeedToTicks(fValue, 
  			m_pGarciaInternal->distanceUnitType());
  data[0] = cmdPAD_IO;
  data[1] = aGARCIA_MOTO_PADS_NULL_LVEL;
  aUtil_StoreShort(&data[2], motoSpeed);
  sendStemPacket(aGARCIA_MOTO_ADDR, 4, data);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// damped-speed-right
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDampedSpeedRightProperty::acpGarciaDampedSpeedRightProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_DAMPEDSPEEDR, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_FLOAT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDampedSpeedRightProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  float fValue = pValue->getFloatVal();
  short motoSpeed = aGarciaGeom_SpeedToTicks(fValue, 
  			m_pGarciaInternal->distanceUnitType());
  data[0] = cmdPAD_IO;
  data[1] = aGARCIA_MOTO_PADS_NULL_RVEL;
  aUtil_StoreShort(&data[2], motoSpeed);
  sendStemPacket(aGARCIA_MOTO_ADDR, 4, data);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// heartbeat-callback
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaHeartbeatCallbackProperty::acpGarciaHeartbeatCallbackProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_HBCALLBACK, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_CALLBACK))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaHeartbeatCallbackProperty::setValue (
  const acpValue* pValue
)
{
  // clean up any old callback objects if present
  if (m_pGarciaInternal->m_pcHBCallback)
    delete m_pGarciaInternal->m_pcHBCallback;
 
  m_pGarciaInternal->m_pcHBCallback = 
  	(acpCallback*)pValue->getCallbackPtrVal();
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// heartbeat-status
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaHeartbeatStatusProperty::acpGarciaHeartbeatStatusProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_HBSTATUS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_BOOL))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaHeartbeatStatusProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(m_pGarciaInternal->m_bHB);
}






/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// (ranger property base class)
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRangerProperty::acpGarciaRangerProperty (
  acpGarciaInternal* pGarciaInternal,
  const char* pName,
  unsigned char cID,
  unsigned char cCode,
  aPROPERTY_FLAGS flags
) :
  acpGarciaProperty(pGarciaInternal, 
  		    pName, 
  	      	    flags),
//  m_cModule(cModule),
  m_cID(cID),
  m_cCode(cCode)
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRangerProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  getRangerValue(m_cID, m_cCode, pValue);
}


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// down-ranger-left
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDownRangerLeftProperty::acpGarciaDownRangerLeftProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal, 
			  aGARCIA_PROPNAME_DOWNRNGLEFT, 
			  aGARCIA_GP_BIT_ENABLEDOWN,
			  aGARCIA_CODE_DOWN_LEFT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_INT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// down-ranger-right
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDownRangerRightProperty::acpGarciaDownRangerRightProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal, 
			  aGARCIA_PROPNAME_DOWNRNGRIGHT, 
			  aGARCIA_GP_BIT_ENABLEDOWN,
			  aGARCIA_CODE_DOWN_RIGHT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_INT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// front-ranger-left
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaFrontRangerLeftProperty::acpGarciaFrontRangerLeftProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal,
			  aGARCIA_PROPNAME_FRONTRNGLEFT,
			  aGARCIA_GP_BIT_ENABLEFRONT,
			  aGARCIA_CODE_FRONT_LEFT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// front-ranger-right
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaFrontRangerRightProperty::acpGarciaFrontRangerRightProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal,
			  aGARCIA_PROPNAME_FRONTRNGRIGHT,
			  aGARCIA_GP_BIT_ENABLEFRONT,
			  aGARCIA_CODE_FRONT_RIGHT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// side-ranger-left
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaSideRangerLeftProperty::acpGarciaSideRangerLeftProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal,
			  aGARCIA_PROPNAME_SIDERNGLEFT,
			  aGARCIA_GP_BIT_ENABLESIDE,
			  aGARCIA_CODE_SIDE_LEFT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// side-ranger-right
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaSideRangerRightProperty::acpGarciaSideRangerRightProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal,
			  aGARCIA_PROPNAME_SIDERNGRIGHT,
			  aGARCIA_GP_BIT_ENABLESIDE,
			  aGARCIA_CODE_SIDE_RIGHT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// rear-ranger-left
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRearRangerLeftProperty::acpGarciaRearRangerLeftProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal,
			  aGARCIA_PROPNAME_REARRNGLEFT,
			  aGARCIA_GP_BIT_ENABLEREAR,
			  aGARCIA_CODE_REAR_LEFT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// rear-ranger-right
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRearRangerRightProperty::acpGarciaRearRangerRightProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerProperty(pGarciaInternal,
			  aGARCIA_PROPNAME_REARRNGRIGHT,
			  aGARCIA_GP_BIT_ENABLEREAR,
			  aGARCIA_CODE_REAR_RIGHT,
			  (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ | aPROPERTY_FLAG_FLOAT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// (ranger threshold property base class)
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRangerThresholdProperty::acpGarciaRangerThresholdProperty (
  acpGarciaInternal* pGarciaInternal,
  const char* pName,
  eGarciaRangerType rangerType,
  unsigned char cModule,
  unsigned char cPadIndex
) :
  acpGarciaProperty(pGarciaInternal, 
  		    pName, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_FLOAT)),
  m_rangerType(rangerType),
  m_cModule(cModule),
  m_cPadIndex(cPadIndex)
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRangerThresholdProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  short srange =
      (short)aGarciaGeom_RangerToRaw(pValue->getFloatVal(),
				     m_rangerType,
				     m_pGarciaInternal->distanceUnitType());
  data[0] = cmdPAD_IO;
  data[1] = (char)m_cPadIndex;
  aUtil_StoreShort(&data[2], srange);
  sendStemPacket(m_cModule, 4, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRangerThresholdProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  char shortStuff[2];
  float fValue;
  shortStuff[0] = getPadValue(m_cModule, 
                              m_cPadIndex);
  shortStuff[1] = getPadValue(m_cModule, 
                              (unsigned char)(m_cPadIndex + 1));
  short srange = aUtil_RetrieveShort(shortStuff);
  fValue = aGarciaGeom_ScaleRangerData((unsigned short)srange,
					 m_rangerType,
					 m_pGarciaInternal->distanceUnitType());
  pValue->set(fValue);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// front-ranger-threshold
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaFrontRangerThresholdProperty::acpGarciaFrontRangerThresholdProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerThresholdProperty(pGarciaInternal,
				   aGARCIA_PROPNAME_FRONTRANGETHR,
				   kGarciaGeomFrontRanger,
				   aGARCIA_MOTO_ADDR,
				   aGARCIA_MOTO_PADS_FRONTTHR)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// side-ranger-threshold
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaSideRangerThresholdProperty::acpGarciaSideRangerThresholdProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerThresholdProperty(pGarciaInternal,
				   aGARCIA_PROPNAME_SIDERANGETHR,
				   kGarciaGeomSideRanger,
				   aGARCIA_MOTO_ADDR,
				   aGARCIA_MOTO_PADS_SIDETHR)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// rear-ranger-threshold
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRearRangerThresholdProperty::acpGarciaRearRangerThresholdProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerThresholdProperty(pGarciaInternal,
				   aGARCIA_PROPNAME_REARRANGETHR,
				   kGarciaGeomRearRanger,
				   aGARCIA_GP_ADDR,
				   32 + aGARCIA_GP_CTR_REARTHR * 2)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// (ranger enable property base class)
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRangerEnableProperty::acpGarciaRangerEnableProperty (
  acpGarciaInternal* pGarciaInternal,
  const char* pName,
  char cBit
) :
  acpGarciaProperty(pGarciaInternal, 
  		    pName, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_INT)),
  m_cBit(cBit)
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRangerEnableProperty::setValue (
  const acpValue* pValue
)
{
  char data[3];
  char mask = (char)(1 << m_cBit);

  data[0] = cmdPAD_IO;
  data[1] = aGARCIA_GP_PADB_MIRRORIO;
  data[2] = (pValue->getIntVal()) ? mask : (char)0;
  data[3] = (char)(~mask);
  sendStemPacket(aGARCIA_GP_ADDR, 4, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRangerEnableProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  int nValue;
  int bits;
  bits = getPadValue(aGARCIA_GP_ADDR, aGARCIA_GP_PADB_MIRRORIO) 
         & (1 << m_cBit);
  nValue = (bits) ? 1 : 0;
  pValue->set(nValue);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// down-ranger-enable
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDownRangerEnableProperty::acpGarciaDownRangerEnableProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerEnableProperty(pGarciaInternal,
				aGARCIA_PROPNAME_DOWNRNGENA,
				aGARCIA_GP_BIT_ENABLEDOWN)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// front-ranger-enable
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaFrontRangerEnableProperty::acpGarciaFrontRangerEnableProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerEnableProperty(pGarciaInternal,
				aGARCIA_PROPNAME_FRONTRNGENA,
				aGARCIA_GP_BIT_ENABLEFRONT)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// side-ranger-enable
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaSideRangerEnableProperty::acpGarciaSideRangerEnableProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerEnableProperty(pGarciaInternal,
				aGARCIA_PROPNAME_SIDERNGENA,
				aGARCIA_GP_BIT_ENABLESIDE)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// rear-ranger-enable
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRearRangerEnableProperty::acpGarciaRearRangerEnableProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaRangerEnableProperty(pGarciaInternal,
				aGARCIA_PROPNAME_REARRNGENA,
				aGARCIA_GP_BIT_ENABLEREAR)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// distance-units
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDistanceUnitsProperty::acpGarciaDistanceUnitsProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_DISTUNITS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceUnitsProperty::setValue (
  const acpValue* pValue
)
{
  m_pGarciaInternal->setDistanceUnitType(pValue->getIntVal());
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceUnitsProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(m_pGarciaInternal->distanceUnitType());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// distance-units-string
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDistanceUnitsStringProperty::acpGarciaDistanceUnitsStringProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_DISTUNITSSTR, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_STRING))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceUnitsStringProperty::setValue (
  const acpValue* pValue
)
{
  if (!aStringCompare(pValue->getStringVal(), aGARCIA_TEXT_METERS)) {
    m_pGarciaInternal->setAngleUnitType(aGARCIA_DISTANCE_METERS);
  } else if (!aStringCompare(pValue->getStringVal(), aGARCIA_TEXT_FEET)) {
    m_pGarciaInternal->setAngleUnitType(aGARCIA_DISTANCE_FEET);
  } else if (!aStringCompare(pValue->getStringVal(), aGARCIA_TEXT_INCHES)) {
    m_pGarciaInternal->setAngleUnitType(aGARCIA_DISTANCE_INCHES);
  }
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceUnitsStringProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  switch (m_pGarciaInternal->distanceUnitType()) {

    case aGARCIA_DISTANCE_METERS:
      pValue->set("meters");
      break;

    case aGARCIA_DISTANCE_FEET:
      pValue->set("feet");
      break;

    case aGARCIA_DISTANCE_INCHES:
      pValue->set("inches");
      break;

  } // switch 
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// angle-units
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaAngleUnitsProperty::acpGarciaAngleUnitsProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_ANGLEUNITS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaAngleUnitsProperty::setValue (
  const acpValue* pValue
)
{
  m_pGarciaInternal->setAngleUnitType(pValue->getIntVal());
}

/////////////////////////////////////////////////////////////////////

void acpGarciaAngleUnitsProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(m_pGarciaInternal->angleUnitType());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// angle-units-string
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaAngleUnitsStringProperty::acpGarciaAngleUnitsStringProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_ANGLEUNITSSTR, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_STRING))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaAngleUnitsStringProperty::setValue (
  const acpValue* pValue
)
{
  if (!aStringCompare(pValue->getStringVal(), aGARCIA_TEXT_RADIANS)) {
    m_pGarciaInternal->setAngleUnitType(aGARCIA_ANGLE_RADIANS);
  } else if (!aStringCompare(pValue->getStringVal(), aGARCIA_TEXT_DEGREES)) {
    m_pGarciaInternal->setAngleUnitType(aGARCIA_ANGLE_DEGREES);
  }
}

/////////////////////////////////////////////////////////////////////

void acpGarciaAngleUnitsStringProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  switch (m_pGarciaInternal->angleUnitType()) {

    case aGARCIA_ANGLE_RADIANS:
      pValue->set("radians");
      break;

    case aGARCIA_ANGLE_DEGREES:
      pValue->set("degrees");
      break;

  } // switch 
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// ir-transmit
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaIRTransmitProperty::acpGarciaIRTransmitProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_IRTRANSMIT, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaIRTransmitProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  data[0] = cmdIRP_XMIT;
  data[1] = aGARCIA_GP_DIRCOMM_TX;
  aUtil_StoreShort(&data[2], (short)pValue->getIntVal());
  sendStemPacket(aGARCIA_GP_ADDR, 4, data);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// ir-receive
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaIRReceiveProperty::acpGarciaIRReceiveProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_IRRECEIVE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaIRReceiveProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  data[0] = cmdCTR_SET;
  data[1] = aGARCIA_MOTO_CTR_IRRX;
  aUtil_StoreShort(&data[2], (short)pValue->getIntVal());
  sendStemPacket(aGARCIA_MOTO_ADDR, 4, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaIRReceiveProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set((int)getCounterValue(aGARCIA_MOTO_ADDR, 
  				   aGARCIA_MOTO_CTR_IRRX));
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// user-flags
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaUserFlagsProperty::acpGarciaUserFlagsProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_EXEFLAGS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaUserFlagsProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  data[0] = cmdPAD_IO;
  data[1] = aGARCIA_MOTO_PADB_EXEFLAGS;
  data[2] = (char)pValue->getIntVal();
  sendStemPacket(aGARCIA_MOTO_ADDR, 3, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaUserFlagsProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(getPadValue(aGARCIA_MOTO_ADDR, 
  			  aGARCIA_MOTO_PADB_EXEFLAGS));
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// status (for manually writing abort code)
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaStatusProperty::acpGarciaStatusProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_STATUS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaStatusProperty::setValue (
  const acpValue* pValue
)
{
  m_pGarciaInternal->abortCurrentBehavior(pValue->getIntVal());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// distance-left
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDistanceLeftProperty::acpGarciaDistanceLeftProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_DISTLEFT, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_FLOAT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceLeftProperty::setValue (
  const acpValue* pValue
)
{
  long nlong;
  char data[aSTEMMAXPACKETBYTES];
  data[0] = cmdMO_ENC32;
  data[1] = aGARCIA_MOTO_MOTOR_LEFT;
  nlong = (long)(pValue->getFloatVal() 
  		 * m_pGarciaInternal->ticksPerUnitDistance());
  aUtil_StoreLong(&data[2], nlong);
  sendStemPacket(aGARCIA_MOTO_ADDR, 6, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceLeftProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(getOdometerValue(aGARCIA_MOTO_ADDR, 
                               aGARCIA_MOTO_MOTOR_LEFT));
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// distance-right
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaDistanceRightProperty::acpGarciaDistanceRightProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_DISTRIGHT, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_FLOAT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceRightProperty::setValue (
  const acpValue* pValue
)
{
  long nlong;
  char data[aSTEMMAXPACKETBYTES];
  data[0] = cmdMO_ENC32;
  data[1] = aGARCIA_MOTO_MOTOR_RIGHT;
  nlong = (long)(pValue->getFloatVal() 
  		 * m_pGarciaInternal->ticksPerUnitDistance());
  aUtil_StoreLong(&data[2], nlong);
  sendStemPacket(aGARCIA_MOTO_ADDR, 6, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaDistanceRightProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(getOdometerValue(aGARCIA_MOTO_ADDR, 
                               aGARCIA_MOTO_MOTOR_RIGHT));
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// stall threshold
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaStallThresholdProperty::acpGarciaStallThresholdProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_STALLTHR, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaStallThresholdProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  short stallThr;
  stallThr = (aShort)pValue->getIntVal();

  if (stallThr < 2) stallThr = 2;

  // NOTE:  new GP and MOTO support multi-byte pad writes
  data[0] = cmdPAD_IO;
  data[1] = aGARCIA_MOTO_PADS_STALLTHR;
  aUtil_StoreShort(&data[2], stallThr);
  sendStemPacket(aGARCIA_MOTO_ADDR, 4, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaStallThresholdProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  char shortStuff[2];
  short nValue;
  shortStuff[0] = getPadValue(aGARCIA_MOTO_ADDR, 
                              aGARCIA_MOTO_PADS_STALLTHR);
  shortStuff[1] = getPadValue(aGARCIA_MOTO_ADDR, 
                              aGARCIA_MOTO_PADS_STALLTHR + 1);
  nValue = aUtil_RetrieveShort(shortStuff);
  pValue->set(nValue);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// stall queue size
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaStallQueueSizeProperty::acpGarciaStallQueueSizeProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_STALLQSIZE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaStallQueueSizeProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  int nBitq;
  short sQmask;
  nBitq = pValue->getIntVal();

  if (nBitq < 1) nBitq = 1;
  if (nBitq > 16) nBitq = 16;
  nBitq = (1 << nBitq) - 1;
  sQmask = (aShort)(nBitq & 0xFFFF);

  data[0] = cmdCTR_SET;
  data[1] = aGARCIA_MOTO_CTR_STALLMASK;
  aUtil_StoreShort(&data[2], sQmask);
  sendStemPacket(aGARCIA_MOTO_ADDR, 4, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaStallQueueSizeProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  int n;
  int k = 0;
  n = (int)getCounterValue(aGARCIA_MOTO_ADDR, 
			   aGARCIA_MOTO_CTR_STALLMASK);

  // convert mask to number of bits set
  while (n) {
    n >>= 1;
    k++;
  }
  pValue->set(k);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// speed
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaSpeedProperty::acpGarciaSpeedProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_SPEED, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_FLOAT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaSpeedProperty::setValue (
  const acpValue* pValue
)
{
  char data[aSTEMMAXPACKETBYTES];
  short motoSpeed;
  float fValue = pValue->getFloatVal();

  // NOTE:  new GP and MOTO support multi-byte pad writes

  float fNewSpeed = fValue;
  // primitive speed is an absolute quantity
  if (fValue < 0) fNewSpeed = 0;
  motoSpeed =
    aGarciaGeom_SpeedToTicks(fValue,
			     m_pGarciaInternal->distanceUnitType());
  aUtil_StoreShort(&data[2], motoSpeed);
  data[0] = cmdPAD_IO;
  data[1] = aGARCIA_MOTO_PADS_DEFVEL;
  sendStemPacket(aGARCIA_MOTO_ADDR, 4, data);
}

/////////////////////////////////////////////////////////////////////

void acpGarciaSpeedProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  char shortStuff[2];
  float fValue;
  shortStuff[0] = getPadValue(aGARCIA_MOTO_ADDR, 
                              aGARCIA_MOTO_PADS_DEFVEL);
  shortStuff[1] = getPadValue(aGARCIA_MOTO_ADDR, 
                              aGARCIA_MOTO_PADS_DEFVEL + 1);
  short motoSpeed = aUtil_RetrieveShort(shortStuff);
  fValue = aGarciaGeom_TicksToSpeed(motoSpeed,
				    m_pGarciaInternal->distanceUnitType());
  pValue->set(fValue);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// status-stream
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaStatusStreamProperty::acpGarciaStatusStreamProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_STATUSSTREAM, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_VOIDPTR))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaStatusStreamProperty::setValue(
  const acpValue* pValue
)
{
  m_pGarciaInternal->m_statusStream = 
  		(aStreamRef)pValue->getVoidPtrVal();
}

/////////////////////////////////////////////////////////////////////

void acpGarciaStatusStreamProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((void*)m_pGarciaInternal->m_statusStream);
}





/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// error-stream
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaErrorStreamProperty::acpGarciaErrorStreamProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_ERRORSTREAM, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_VOIDPTR))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaErrorStreamProperty::setValue(
  const acpValue* pValue
)
{
  m_pGarciaInternal->m_errorStream = 
  		(aStreamRef)pValue->getVoidPtrVal();
}

/////////////////////////////////////////////////////////////////////

void acpGarciaErrorStreamProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((void*)m_pGarciaInternal->m_errorStream);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// relay-index
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRelayIndexProperty::acpGarciaRelayIndexProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_RELAYINDEX, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRelayIndexProperty::setValue(
  const acpValue* pValue
)
{
  int i = pValue->getIntVal();
  if (i >= 2 && i <=254) {
    m_pGarciaInternal->m_nRelayIndex = i;
    if (!m_pGarciaInternal->m_relayStreams[i>>1]) {
      aErr err = aErrNone;
      aStem_CreateRelayStream(
        m_pGarciaInternal->m_stemRef,
        (unsigned char)i,
        &m_pGarciaInternal->m_relayStreams[i>>1],
        &err);
      aAssert(err == aErrNone);
    }
  }
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRelayIndexProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((int)m_pGarciaInternal->m_nRelayIndex);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// relay-status
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRelayStatusProperty::acpGarciaRelayStatusProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_RELAYSTATUS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRelayStatusProperty::setValue(
  const acpValue* pValue
)
{
  m_pGarciaInternal->m_nRelayStatus =
    pValue->getIntVal();
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRelayStatusProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((int)m_pGarciaInternal->m_nRelayStatus);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// relay-byte
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaRelayByteProperty::acpGarciaRelayByteProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_RELAYBYTE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  			            | aPROPERTY_FLAG_INT))
{
}

/////////////////////////////////////////////////////////////////////

void acpGarciaRelayByteProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  aErr recErr = aErrNone;
  char c = 0;
  int i = m_pGarciaInternal->m_nRelayIndex;

  if (i >= 2 && i <=254) {
    aStream_Read(
      m_pGarciaInternal->m_ioRef, 
      m_pGarciaInternal->m_relayStreams[i>>1],
      &c,
      1,
      &recErr);
  } else {
    recErr = aErrParam;
  }

  m_pGarciaInternal->m_nRelayStatus = recErr;
  pValue->set((int)c);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// servo 0, 1, 2, 3
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaServo0Property::acpGarciaServo0Property (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_SERVO0, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_OBJECT))
{
}

acpGarciaServo1Property::acpGarciaServo1Property (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_SERVO1, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_OBJECT))
{
}

acpGarciaServo2Property::acpGarciaServo2Property (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_SERVO2, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_OBJECT))
{
}

acpGarciaServo3Property::acpGarciaServo3Property (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_SERVO3, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_OBJECT))
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// camera boom
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpGarciaCameraBoomProperty::acpGarciaCameraBoomProperty (
  acpGarciaInternal* pGarciaInternal
) :
  acpGarciaProperty(pGarciaInternal, 
  		    aGARCIA_PROPNAME_CAMERABOOM, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ
  			            | aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_OBJECT))
{
}
