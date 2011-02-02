/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: aGarciaProperties.h                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia Properties.               //
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

#ifndef _acpGaricaProperties_H_
#define _acpGaricaProperties_H_


#define aGARCIA_PRIMITIVE_NAME		"primitive-name"

// property names for the Garcia objects
#define aGARCIA_PROPNAME_ACTIVE		"active"
#define aGARCIA_PROPNAME_IDLE		"idle"
#define aGARCIA_PROPNAME_LED		"user-led"
#define aGARCIA_PROPNAME_BUTTON		"user-button"
#define aGARCIA_PROPNAME_DOWNRNGENA	"down-ranger-enable"
#define aGARCIA_PROPNAME_SIDERNGENA	"side-ranger-enable"
#define aGARCIA_PROPNAME_FRONTRNGENA	"front-ranger-enable"
#define aGARCIA_PROPNAME_REARRNGENA	"rear-ranger-enable"
#define aGARCIA_PROPNAME_DOWNRNGLEFT	"down-ranger-left"
#define aGARCIA_PROPNAME_DOWNRNGRIGHT	"down-ranger-right"
#define aGARCIA_PROPNAME_IRRECEIVE	"ir-receive"
#define aGARCIA_PROPNAME_IRTRANSMIT	"ir-transmit"
#define aGARCIA_PROPNAME_STATUS		"status"
#define	aGARCIA_PROPNAME_EXEFLAGS	"user-flags"
#define aGARCIA_PROPNAME_HBCALLBACK     "heartbeat-callback"
#define aGARCIA_PROPNAME_HBSTATUS       "heartbeat-status"
#define aGARCIA_PROPNAME_APPOBJECT	"app-object"

#define aGARCIA_PROPNAME_STALLTHR	"stall-threshold"
#define aGARCIA_PROPNAME_STALLQSIZE	"stall-queue-size"

#define aGARCIA_PROPNAME_SPEED		"speed"
#define aGARCIA_PROPNAME_SIDERNGLEFT	"side-ranger-left"
#define aGARCIA_PROPNAME_SIDERNGRIGHT	"side-ranger-right"
#define aGARCIA_PROPNAME_FRONTRNGLEFT	"front-ranger-left"
#define aGARCIA_PROPNAME_FRONTRNGRIGHT	"front-ranger-right"
#define aGARCIA_PROPNAME_REARRNGLEFT	"rear-ranger-left"
#define aGARCIA_PROPNAME_REARRNGRIGHT	"rear-ranger-right"
#define aGARCIA_PROPNAME_DISTLEFT	"distance-left"
#define aGARCIA_PROPNAME_DISTRIGHT	"distance-right"
#define aGARCIA_PROPNAME_FRONTRANGETHR	"front-ranger-threshold"
#define aGARCIA_PROPNAME_SIDERANGETHR	"side-ranger-threshold"
#define aGARCIA_PROPNAME_REARRANGETHR	"rear-ranger-threshold"
#define aGARCIA_PROPNAME_DAMPEDSPEEDL	"damped-speed-left"
#define aGARCIA_PROPNAME_DAMPEDSPEEDR	"damped-speed-right"

#define aGARCIA_PROPNAME_NAME		"name"
#define aGARCIA_PROPNAME_UNIQUEID	"unique-id"
#define aGARCIA_PROPNAME_COMPSTATUS	"completion-status"
#define aGARCIA_PROPNAME_EXPECTSTATUS	"expected-status"
#define aGARCIA_PROPNAME_COMPCALLBACK	"completion-callback"
#define aGARCIA_PROPNAME_EXECCALLBACK	"execute-callback"
#define aGARCIA_PROPNAME_INPUTSTREAM	"input-stream"
#define aGARCIA_PROPNAME_STATUSSTREAM	"status-stream"
#define aGARCIA_PROPNAME_ERRORSTREAM	"error-stream"
#define aGARCIA_PROPNAME_RELAYINDEX	"relay-index"
#define aGARCIA_PROPNAME_RELAYBYTE	"relay-byte"
#define aGARCIA_PROPNAME_RELAYSTATUS	"relay-status"

#define aGARCIA_PROPNAME_DISTUNITS	"distance-units"
#define aGARCIA_PROPNAME_DISTUNITSSTR	"distance-units-string"
#define aGARCIA_PROPNAME_ANGLEUNITS	"angle-units"
#define aGARCIA_PROPNAME_ANGLEUNITSSTR	"angle-units-string"

#define aGARCIA_PROPNAME_BATTERYLEVEL	"battery-level"
#define aGARCIA_PROPNAME_BATTERYVOLTAGE	"battery-voltage"

#define aGARCIA_PROPNAME_SERVO0		"obj-servo-0"
#define aGARCIA_PROPNAME_SERVO1		"obj-servo-1"
#define aGARCIA_PROPNAME_SERVO2		"obj-servo-2"
#define aGARCIA_PROPNAME_SERVO3		"obj-servo-3"

#define	aGARCIA_PROPNAME_CAMERABOOM	"obj-camera-boom"

#endif // _acpGaricaProperties_H_

