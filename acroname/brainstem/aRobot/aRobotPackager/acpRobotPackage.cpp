/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotPackage.cpp 	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of robot API data package           //
//		routines.					   //
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

#include "aVersion.h"
#include "aIO.h"

#include "acpRobotPackage.h"

#include "acpTag_apiPACKAGE.h"
#include "acpTag_PROPERTY.h"
#include "acpTag_PRIMITIVE.h"
#include "acpTag_TEMPLATE.h"
#include "acpTag_USEROBJECT.h"
#include "acpTag_SET.h"
#include "acpTag_GET.h"
#include "acpTag_INITIALIZER.h"
#include "acpTag_LINK.h"



/////////////////////////////////////////////////////////////////////

acpRobotPackage::acpRobotPackage(
  aIOLib ioRef,
  aStreamRef stream // = NULL
) :
  acpPackage(NULL,
  	     ioRef, 
  	     aVERSION_MAJOR, 
  	     aVERSION_MINOR, 
  	     aPACKAGE_BUILD_NUM,
  	     aROBOT_PKG_FILEVERSION, 
  	     stream)
{
}



/////////////////////////////////////////////////////////////////////

void acpRobotPackage::initialize()
{
  // assemble all the tags the package currently can recognize
  addRecognizedTag(new acpTag_apiPACKAGE());
  addRecognizedTag(new acpTag_PROPERTY());
  addRecognizedTag(new acpTag_PRIMITIVE());
  addRecognizedTag(new acpTag_TEMPLATE());
  addRecognizedTag(new acpTag_USEROBJECT());
  addRecognizedTag(new acpTag_SET());
  addRecognizedTag(new acpTag_GET());
  addRecognizedTag(new acpTag_INITIALIZER());
  addRecognizedTag(new acpTag_LINK());
  
  acpPackage::initialize();
}
