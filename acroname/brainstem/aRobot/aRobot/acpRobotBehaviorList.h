/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotBehaviorList.h                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Robot API behavior object.       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//  Behaviors parametrize primitives.  They can also act as        //
//  containers for other sets of behaviors.                        //
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

#ifndef _acpRobotBehaviorList_H_
#define _acpRobotBehaviorList_H_

#include "acpRobotBehavior.h"
#include "acpList.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpObject.h"
#include "acpRobotPrimitive.h"
#include "acpRobotInternal.h"


class acpRobot;
class acpRobotBehaviorListFilenameProperty;

typedef aErr (*aBehaviorCallbackProc)(acpRobot* pcRobot,
				      void* behavior);

class acpRobotBehaviorList :
  public acpRobotBehavior
{

  public:
  				acpRobotBehaviorList(
  				  const char* pName,
  				  acpRobotPrimitive* pPrimitive,
  				  const int nID,
  				  acpRobotInternal* pcRobotInternal,
  				  acpRobotBehaviorList* pParent = NULL);

	  			~acpRobotBehaviorList();


    void			addChildBehavior(
    				  acpRobotBehavior* pChild);
    void			dyingChild(const short status,
					   bool bSuccess);

    virtual void		setNamedValue (
				  const char* pPropertyName,
				  const acpValue* pValue);
    virtual void		setValue(
				  const int nPropIndex,
				  const acpValue* pValue);

  protected:

    virtual bool		finish(
    				  const short status,
			          bool* pbSuccess);

  private:
    int				m_nRefCount;
    short			m_nFinalStatus;
    bool			m_bResult;
    bool			m_bNormalTermination;

    acpList<acpRobotBehavior>	m_childBehaviors;

  friend class acpRobotInternal;
  friend class acpRobotGlobal;
  friend class acpRobotScript;
};

#endif // _acpRobotBehaviorList_H_
