/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: aRobotProperties.h                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the core Robot API Properties.       //
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

#ifndef _aRobotProperties_H_
#define _aRobotProperties_H_

// API TEA program constants

#define	aROBOT_APITEA_PROCESSCT		2
#define	aROBOT_APITEA_STACKSIZE		128
#define	aROBOT_APITEA_PROPPROCID	0
#define	aROBOT_APITEA_PRIMPROCID	1
#define aROBOT_APITEA_NOTFOUND		-1

// core primitive names

#define aROBOT_PRIMNAME_GLOBAL		"global"

// core property names for the robot object
// core property names for the primitives

#define aROBOT_PROPNAME_ACTIVE		"active"
#define aROBOT_PROPNAME_IDLE		"idle"
#define aROBOT_PROPNAME_STATUSSTREAM	"status_stream"
#define aROBOT_PROPNAME_ERRORSTREAM	"error_stream"
#define aROBOT_PROPNAME_HBCALLBACK	"heartbeat_callback"
#define aROBOT_PROPNAME_HBSTATUS	"heartbeat_status"

#define aROBOT_PROPNAME_NAME		"name"
#define aROBOT_PROPNAME_UNIQUEID	"unique_id"
#define aROBOT_PROPNAME_PRIMITIVENAME	"primitive_name"
#define aROBOT_PROPNAME_EXECCALLBACK	"execute_callback"
#define aROBOT_PROPNAME_COMPCALLBACK	"completion_callback"
#define aROBOT_PROPNAME_COMPSTATUS	"completion_status"
#define aROBOT_PROPNAME_EXPECTSTATUS	"expected_status"
#define aROBOT_PROPNAME_PARAMHTML	"param_html"
#define aROBOT_PROPNAME_MODULE		"module"
#define aROBOT_PROPNAME_FILESLOT	"file_slot"
#define aROBOT_PROPNAME_PROCESSID	"process_id"
#define aROBOT_PROPNAME_DATAPTR		"dataptr"
#define aROBOT_PROPNAME_TEMPLATENAME	"template_name"

#define aROBOT_PROPNAME_DISTUNITS	"distance_units"
#define aROBOT_PROPNAME_DISTUNITSSTR	"distance_units_string"
#define aROBOT_PROPNAME_ANGLEUNITS	"angle_units"
#define aROBOT_PROPNAME_ANGLEUNITSSTR	"angle_units_string"
#define aROBOT_PROPNAME_MASSUNITS	"mass_units"
#define aROBOT_PROPNAME_MASSUNITSSTR	"mass_units_string"

#endif // _aRobotProperties_H_
