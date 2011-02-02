/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpBehavior.h                                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API behavior object.      //
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

#ifndef _acpBehavior_H_
#define _acpBehavior_H_

#include "acpObject.h"
#include "acpList.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpGarciaPrimitive.h"
#include "acpGarciaInternal.h"


class acpInt {
  public:
  		acpInt(int val = 0) : m_val(val) {}
    virtual	~acpInt() {}
  
    int		m_val;
};


class acpGarcia;
class acpBehaviorList;
class acpBehaviorProperty;

class acpBehavior :
  public acpObject
{
  protected:
  				acpBehavior(
  				  const char* pName,
  				  acpGarciaPrimitive* pPrimitive,
  				  const int nID,
  				  acpGarciaInternal* pcGarciaInternal,
  				  acpBehaviorList* pParent = NULL);

  public:
    virtual   			~acpBehavior();

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
    acpBehaviorList*		m_pParent;
    bool			m_bNormalTermination;

  private:
    static aErr			propertyCloneProc(
    				 const char* pName,
				 const int nIndex,
				 aPROPERTY_FLAGS flags,
				 void* vpRef);

    int				m_nPrimitiveNamePropIndex;

    short			m_nFinalStatus;
    acpGarciaInternal*		m_pcGarciaInternal;
    acpGarciaPrimitive*		m_pPrimitive;
    bool			m_bRunning;
    bool			m_bResult;

    acpList<acpInt>		m_expectedStatus;
    acpList<acpInt>		m_globalIndex;
    acpList<acpValue>		m_globalValue;

  friend class acpGarciaPrimitive;
  friend class acpGarciaInternal;
  friend class acpGarciaGlobal;
  friend class acpGarciaSleep;
  friend class acpBehaviorList;
  friend class acpGarciaXMLScript;
  friend class acpBehaviorProperty;
};



/////////////////////////////////////////////////////////////////////

class acpBehaviorProperty : public acpProperty
{
  public:
  				acpBehaviorProperty(
  				  acpGarciaPrimitive* pPrimitive,
  				  const char* pName,
  				  aPROPERTY_FLAGS flags);
  protected:

    acpGarciaPrimitive*		m_pPrimitive;
};


/////////////////////////////////////////////////////////////////////

class acpBehaviorExecuteCBProperty : public acpBehaviorProperty
{
  public:
  				acpBehaviorExecuteCBProperty(
  				  acpGarciaPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpBehaviorCompletionCBProperty : public acpBehaviorProperty
{
  public:
  				acpBehaviorCompletionCBProperty(
  				  acpGarciaPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpBehaviorStatusProperty : public acpBehaviorProperty
{
  public:
  				acpBehaviorStatusProperty(
  				  acpGarciaPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpBehaviorNameProperty : public acpBehaviorProperty
{
  public:
  				acpBehaviorNameProperty(
  				  acpGarciaPrimitive* pPrimitive);
};


/////////////////////////////////////////////////////////////////////

class acpBehaviorExpectedStatusProperty : public acpBehaviorProperty
{
  public:
  				acpBehaviorExpectedStatusProperty(
  				  acpGarciaPrimitive* pPrimitive);
};


#endif // _acpBehavior_H_
