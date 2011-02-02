/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPackageTagID.h                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of virtual robot data package       	   //
//		tag IDs.					   //
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

#ifndef _acpPackageTagID_H_
#define _acpPackageTagID_H_


/////////////////////////////////////////////////////////////////////
// Package Tags
//   These names must be synchronized with the tag names in
//   acpPackageTag.cpp

#define aTAG_VIEWS		0x0001
#define aTAG_TRIANGLES		0x0002
#define aTAG_SPHERE		0x0003
#define aTAG_PLUGIN		0x0004
#define aTAG_DYNAMIC		0x0005
#define aTAG_PATCHBAY		0x0006
#define aTAG_PLANE		0x0007
#define aTAG_BOX		0x0008
#define aTAG_CYLINDER		0x0009
#define aTAG_CAPSULE		0x000A
#define aTAG_MOTOR		0x000B
#define aTAG_INSTANCE		0x000C
#define aTAG_PROPERTY		0x000D
#define aTAG_SET		0x000E
#define aTAG_GET		0x000F
#define aTAG_PRIMITIVE		0x0010
#define aTAG_TEMPLATE		0x0011
#define aTAG_USEROBJECT		0x0012
#define aTAG_INITIALIZER	0x0013
#define aTAG_apiPACKAGE		0x0014
#define aTAG_LINK		0x0015
#define aTAG_MOTORLINK		0x0016
#define aTAG_LICENSE		0x0017
#define aTAG_SIMULATION		0x0018
#define aTAG_SERVO		0x0019
#define aTAG_POINTS		0x001A

#endif // _acpPackageTagID_H_
