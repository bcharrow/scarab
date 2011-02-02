/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpBehaviorList.h                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API behavior object.      //
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

#ifndef _acpBehaviorList_H_
#define _acpBehaviorList_H_

#include "acpBehavior.h"
#include "acpList.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpObject.h"
#include "acpGarciaPrimitive.h"
#include "acpGarciaInternal.h"


class acpGarcia;
class acpBehaviorListFilenameProperty;

typedef aErr (*aBehaviorCallbackProc)(acpGarcia* pcGarcia,
				      void* behavior);

class acpBehaviorList :
  public acpBehavior
{

  public:
  				acpBehaviorList(
  				  const char* pName,
  				  acpGarciaPrimitive* pPrimitive,
  				  const int nID,
  				  acpGarciaInternal* pcGarciaInternal,
  				  acpBehaviorList* pParent = NULL);

	  			~acpBehaviorList();


    void			addChildBehavior(
    				  acpBehavior* pChild);
    void			dyingChild(const short status,
					   bool bSuccess);

    virtual void		setNamedValue (
				  const char* pPropertyName,
				  const acpValue* pValue);
    virtual void		setValue(
				  const int nPropIndex,
				  const acpValue* pValue);

  protected:

    void			execute();
    virtual bool		finish(
    				  const short status,
			          bool* pbSuccess);

  private:
    int				m_nRefCount;
    short			m_nFinalStatus;
    bool			m_bResult;
    bool			m_bNormalTermination;

    acpList<acpBehavior>	m_childBehaviors;

  friend class acpGarciaInternal;
  friend class acpGarciaGlobal;
  friend class acpGarciaSleep;
  friend class acpGarciaXMLScript;
};

#endif // _acpBehaviorList_H_
