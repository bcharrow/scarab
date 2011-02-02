/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotCoreProperties.cpp                                //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot core properties.       //
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

#include "aUtil.h"
#include "aCmd.tea"
#include "aRobotDefs.tea"
#include "aRobotProperties.h"
#include "acpRobotProperties.h"


/////////////////////////////////////////////////////////////////////

acpRobotProperty::acpRobotProperty (
  acpRobotInternal* pRobotInternal,
  const char* pName,
  aPROPERTY_FLAGS flags,
  const char* pDescription
) :
  acpProperty(pName, flags, pDescription),
  m_pRobotInternal(pRobotInternal)
{
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// active
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotActiveProperty::acpRobotActiveProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_ACTIVE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_BOOL),
  		   "This property is true when the robot has a live"
  		   " link established between the robot API and the"
  		   " robot. Typically, this means there is an active"
  		   " BrainStem link")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotActiveProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{ 
  pValue->set(m_pRobotInternal->isActive());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// idle
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotIdleProperty::acpRobotIdleProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_IDLE, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_BOOL),
  		   "This property is true when there are no active"
  		   " primitives running on the robot.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotIdleProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{ 
  pValue->set(m_pRobotInternal->isIdle());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// heartbeat-callback
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotHeartbeatCallbackProperty::acpRobotHeartbeatCallbackProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_HBCALLBACK, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_CALLBACK),
  	           "Set this property with a callback object to get"
  	           " notified through the callback that a heartbeat"
  	           " has been received.  This is typically used for"
  	           " user interfaces or host code that needs to see"
  	           " the heartbeats.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotHeartbeatCallbackProperty::setValue (
  const acpValue* pValue
)
{
  // clean up any old callback objects if present
  if (m_pRobotInternal->m_pcHBCallback)
    delete m_pRobotInternal->m_pcHBCallback;
 
  m_pRobotInternal->m_pcHBCallback = 
  	(acpCallback*)pValue->getCallbackPtrVal();
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// heartbeat-status
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotHeartbeatStatusProperty::acpRobotHeartbeatStatusProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_HBSTATUS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_BOOL),
  	           "This property can be read to determine the"
  	           " heartbeat state.  It is true when the "
  	           " heartbeat is up, false when it is down.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotHeartbeatStatusProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(m_pRobotInternal->m_bHB);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// distance_units
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotDistanceUnitsProperty::acpRobotDistanceUnitsProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_DISTUNITS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_INT),
  		   "This property determines what the distance"
  		   " units will be for the entire API.  These"
  		   " units are used for al linear distance values"
  		   " both input and output from the API.  These"
  		   " units are determined in the aRobotDefs.tea"
  		   " file.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotDistanceUnitsProperty::setValue (
  const acpValue* pValue
)
{
  m_pRobotInternal->setDistanceUnitType(pValue->getIntVal());
}

/////////////////////////////////////////////////////////////////////

void acpRobotDistanceUnitsProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(m_pRobotInternal->distanceUnitType());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// distance_units_string
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotDistanceUnitsStringProperty::acpRobotDistanceUnitsStringProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_DISTUNITSSTR, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_STRING),
  	           "This property is the text name of the units being"
  	           " used.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotDistanceUnitsStringProperty::setValue (
  const acpValue* pValue
)
{
  if (!aStringCompare(pValue->getStringVal(), aROBOT_TEXT_METERS)) {
    m_pRobotInternal->setDistanceUnitType(aROBOT_UNITS_METERS);
  } else if (!aStringCompare(pValue->getStringVal(), aROBOT_TEXT_FEET)) {
    m_pRobotInternal->setDistanceUnitType(aROBOT_UNITS_FEET);
  } else if (!aStringCompare(pValue->getStringVal(), aROBOT_TEXT_INCHES)) {
    m_pRobotInternal->setDistanceUnitType(aROBOT_UNITS_INCHES);
  }
}

/////////////////////////////////////////////////////////////////////

void acpRobotDistanceUnitsStringProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  switch (m_pRobotInternal->distanceUnitType()) {

    case aROBOT_UNITS_METERS:
      pValue->set("meters");
      break;

    case aROBOT_UNITS_FEET:
      pValue->set("feet");
      break;

    case aROBOT_UNITS_INCHES:
      pValue->set("inches");
      break;

  } // switch 
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// angle_units
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotAngleUnitsProperty::acpRobotAngleUnitsProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_ANGLEUNITS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_INT),
  	           "This property determines how all angular units"
  	           " are handled.  The codes for these units are"
  	           " defined in the aRobotDefs.tea file in the"
  	           " aSystem directory.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotAngleUnitsProperty::setValue (
  const acpValue* pValue
)
{
  m_pRobotInternal->setAngleUnitType(pValue->getIntVal());
}

/////////////////////////////////////////////////////////////////////

void acpRobotAngleUnitsProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(m_pRobotInternal->angleUnitType());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// angle_units_string
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotAngleUnitsStringProperty::acpRobotAngleUnitsStringProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_ANGLEUNITSSTR, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_STRING),
  		   "This property is the text name of the units"
  		   " used for angular values.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotAngleUnitsStringProperty::setValue (
  const acpValue* pValue
)
{
  if (!aStringCompare(pValue->getStringVal(), aROBOT_TEXT_RADIANS)) {
    m_pRobotInternal->setAngleUnitType(aROBOT_UNITS_RADIANS);
  } else if (!aStringCompare(pValue->getStringVal(), aROBOT_TEXT_DEGREES)) {
    m_pRobotInternal->setAngleUnitType(aROBOT_UNITS_DEGREES);
  }
}

/////////////////////////////////////////////////////////////////////

void acpRobotAngleUnitsStringProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  switch (m_pRobotInternal->angleUnitType()) {

    case aROBOT_UNITS_RADIANS:
      pValue->set("radians");
      break;

    case aROBOT_UNITS_DEGREES:
      pValue->set("degrees");
      break;

  } // switch 
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// mass_units
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotMassUnitsProperty::acpRobotMassUnitsProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_MASSUNITS, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_WRITE 
  			            | aPROPERTY_FLAG_INT),
  	           "These property is the code of the mass units"
  	           " being used by the API.  These unit codes are"
  	           " defined in the aRobotDefs.tea file in the"
  	           " aSystem directory.")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotMassUnitsProperty::setValue (
  const acpValue* pValue
)
{
  m_pRobotInternal->setMassUnitType(pValue->getIntVal());
}

/////////////////////////////////////////////////////////////////////

void acpRobotMassUnitsProperty::getValue (
  const acpObject* pObject,
  acpValue* pValue
)
{
  pValue->set(m_pRobotInternal->massUnitType());
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// status_stream
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotStatusStreamProperty::acpRobotStatusStreamProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_STATUSSTREAM, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_VOIDPTR),
  		   "This property is the stream reference that"
  		   " is used for writing all status.  It is of the"
  		   " aStreamRef data type (defined in aIO library).")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotStatusStreamProperty::setValue(
  const acpValue* pValue
)
{
  m_pRobotInternal->m_statusStream = 
  		(aStreamRef)pValue->getVoidPtrVal();
}

/////////////////////////////////////////////////////////////////////

void acpRobotStatusStreamProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((void*)m_pRobotInternal->m_statusStream);
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// error-stream
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

acpRobotErrorStreamProperty::acpRobotErrorStreamProperty (
  acpRobotInternal* pRobotInternal
) :
  acpRobotProperty(pRobotInternal, 
  		    aROBOT_PROPNAME_ERRORSTREAM, 
  	      	    (aPROPERTY_FLAGS)(aPROPERTY_FLAG_WRITE
  			            | aPROPERTY_FLAG_READ 
  			            | aPROPERTY_FLAG_VOIDPTR),
  		   "This property is the stream reference that"
  		   " is used for writing all errors.  It is of the"
  		   " aStreamRef data type (defined in aIO library).")
{
}

/////////////////////////////////////////////////////////////////////

void acpRobotErrorStreamProperty::setValue(
  const acpValue* pValue
)
{
  m_pRobotInternal->m_errorStream = 
  		(aStreamRef)pValue->getVoidPtrVal();
}

/////////////////////////////////////////////////////////////////////

void acpRobotErrorStreamProperty::getValue(
  const acpObject* pObject,
  acpValue* pValue)
{
  pValue->set((void*)m_pRobotInternal->m_errorStream);
}
