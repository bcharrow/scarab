/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_AXIS.h 		                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of XML tag class.                       //
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

#ifndef _acpXmlTag_AXIS_H_
#define _acpXmlTag_AXIS_H_

#include "acpPkgXML.h"

#include "acpXmlTag_VECTOR.h"


class AXIS :
  public VECTOR
{
  public:
  			AXIS(
  			  acpPkgImport* pImporter,
  			  acpPkgXML* pParent = NULL) :
  			    VECTOR(pImporter, pParent)
  			    {}

    acpPkgXML*		clone(
    			  acpPkgXML* pParent = NULL)
    			  { return new AXIS(m_pImporter, 
					    pParent); }

    const char*  	tagName() const
  			  { return "AXIS"; }

    bool		getDescription(
    			  acpString& description) 
    			  { description += 
    			    "Defines an axis of rotation for the parent"
    			    " object.  Axes are typically 5 DOF"
    			    " constraints between two objects or the"
    			    " basis an angular measure between two objects."
    			  ; return true; }

    bool		getChildDescription(
    			  acpPkgXML* pParent,
    			  acpString& description) 
    			  { if (!aStringCompare(pParent->tagName(), 
    			  	                "MOTOR")) {
                            description += 
    			    "Rotational axis of the motor with the"
    			    " direction based on the right-hand rule."
    			    ;return true; } 
    			    else if (!aStringCompare(pParent->tagName(), 
    			  	                "SERVO")) {
                            description += 
    			    "Rotational axis of the servo with the"
    			    " direction based on the right-hand rule."
    			    ;return true; }
    			    return false; }
};

#endif // _acpXmlTag_AXIS_H_
