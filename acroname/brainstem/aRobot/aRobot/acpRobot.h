/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobot.h                                                //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Robot API abstract library       //
//              object.  This acts as a wrapper class to provide   //
//              a clean, and simple interface to the               //
//              hidden acpRobotInternal object.                    //
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

#ifndef _acpRobot_H_
#define _acpRobot_H_

#include "aRobotOSExport.h"
#include "acpObject.h"

class acpRobotInternal;
class acpRobot;

class aROBOT_EXPORT_CLASS acpRobot :
  public acpObject
{
  public:
  				acpRobot(
    				  const char* pName = NULL);
    virtual			~acpRobot();

    // manage robot sub objects
    virtual int			numSubObjects() const;
    virtual void		enumSubObjects(
    				  aObjectEnumProc enumProc,
    				  void* vpRef,
    				  const char* pClassNameFilter = NULL) const;
    virtual acpObject* 		getSubObject(
				  const char* pClassName,
				  const char* pName) const;
    virtual acpObject*		getSubObject (
    				  const int nObjectIndex) const;
  
    // manage robot properties
    virtual void                addProperty(
				  acpProperty* pProperty,
				  acpValue* pDefault = NULL);
    virtual int			numProperties() const;
    virtual void		enumProperties(
    				  propertyEnumProc enumProc,
    				  void* vpRef) const;
    virtual acpProperty*	getProperty(
    				  const int nPropertyIndex) const;
 

    virtual int			getPropertyIndex(
    				  const char* pPropertyName) const;
    virtual char*		getPropertyName(
    				  const int nPropertyIndex) const;
    virtual aPROPERTY_FLAGS	getPropertyFlags(
    				  const int nPropertyIndex) const;


    virtual acpValue*		getValue(
    				  const int nPropIndex);
    virtual acpValue*		getNamedValue(
    				  const char* pPropName);

    virtual void		setValue(
    				  const int nPropIndex,
    				  const acpValue* pValue);
    virtual void		setNamedValue(
    				  const char* pPropName,
    				  const acpValue* pValue);

    // behavior creation/destruction
    virtual acpObject*		createNamedBehavior(
    				  const char* pPrimitiveName,
    				  const char* pBehaviorName);
    virtual acpObject*		createBehavior(
    				  const int nPrimitiveIndex,
    				  const char* pBehaviorName);

    // behavior processing routines
    virtual void		queueBehavior(
    				  acpObject* pBehavior);
    virtual void		flushQueuedBehaviors();
    virtual int			handleCallbacks(
    				  const unsigned long nMSYield = 0);

    // status message formatting
    virtual char*		statusString(
    				  const short nStatus,
    				  char* pText,
    				  unsigned int nMaxChars);

  private:
    acpRobotInternal*		m_pInternal;
};


#endif // _acpRobot_H_
