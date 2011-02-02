/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotBehavior.h                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Robot API behavior object.       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//  Behaviors parameterize primitives.    .                        //
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

#ifndef _acpRobotBehavior_H_
#define _acpRobotBehavior_H_

#include "acpObject.h"
#include "acpList.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpRobotPrimitive.h"


class acpInt {
  public:
  		acpInt(int val = 0) : m_val(val) {}
    virtual	~acpInt() {}
  
    int		m_val;
};



class acpRobotBehavior :
  public acpObject
{
  protected:
  				acpRobotBehavior(
  				  const char* pName,
  				  acpRobotPrimitive* pPrimitive,
  				  const int nID,
  				  acpRobotInternal* pcRobotInternal,
  				  acpRobotBehaviorList* pParent = NULL);

  public:
    virtual   			~acpRobotBehavior();

    virtual aErr		writeToStream(
    				  const aStreamRef stream) const;

    virtual void		setNamedValue (
				  const char* pPropertyName,
				  const acpValue* pValue);
    virtual void		setValue(
				  const int nPropIndex,
				  const acpValue* pValue);

    virtual bool		isRunning()
    				  { return m_bRunning; }

    virtual void		getDescription(acpTextBuffer& buffer);

  protected:

    virtual void		execute();
    virtual bool		finish(
    				  const short status,
			          bool* pbSuccess);
    acpRobotBehaviorList*		m_pParent;
    bool			m_bNormalTermination;

  private:
    static aErr			propertyCloneProc(
    				 const char* pName,
				 const int nIndex,
				 aPROPERTY_FLAGS flags,
				 void* vpRef);

    int				m_nPrimitiveNamePropIndex;

    short			m_nFinalStatus;
    acpRobotInternal*		m_pcRobotInternal;
    acpRobotPrimitive*		m_pPrimitive;
    bool			m_bRunning;
    bool			m_bResult;

    acpList<acpInt>		m_expectedStatus;
    acpList<acpInt>		m_globalIndex;
    acpList<acpValue>		m_globalValue;

  friend class acpRobotPrimitive;
  friend class acpRobotInternal;
  friend class acpRobotGlobal;
  friend class acpRobotScript;
  friend class acpRobotSleep;
  friend class acpRobotBehaviorList;
  friend class acpRobotBehaviorProperty;
};



/////////////////////////////////////////////////////////////////////

class acpRobotBehaviorProperty : public acpProperty
{
  public:
  				acpRobotBehaviorProperty(
  				  acpRobotPrimitive* pPrimitive,
  				  const char* pName,
  				  aPROPERTY_FLAGS flags);
  protected:

    acpRobotPrimitive*		m_pPrimitive;
};


/////////////////////////////////////////////////////////////////////

class acpRobotBehaviorExecuteCBProperty : public acpRobotBehaviorProperty
{
  public:
  				acpRobotBehaviorExecuteCBProperty(
  				  acpRobotPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpRobotBehaviorCompletionCBProperty : public acpRobotBehaviorProperty
{
  public:
  				acpRobotBehaviorCompletionCBProperty(
  				  acpRobotPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpRobotBehaviorStatusProperty : public acpRobotBehaviorProperty
{
  public:
  				acpRobotBehaviorStatusProperty(
  				  acpRobotPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpRobotBehaviorNameProperty : public acpRobotBehaviorProperty
{
  public:
  				acpRobotBehaviorNameProperty(
  				  acpRobotPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpRobotBehaviorExpectedStatusProperty : public acpRobotBehaviorProperty
{
  public:
  				acpRobotBehaviorExpectedStatusProperty(
  				  acpRobotPrimitive* pPrimitive);
};




/////////////////////////////////////////////////////////////////////

class acpRobotBehaviorParamHTMLProperty: public acpRobotBehaviorProperty
{
  public:
		  		acpRobotBehaviorParamHTMLProperty (
  				  acpRobotPrimitive* pPrimitive);
};

#endif // _acpRobotBehavior_H_
