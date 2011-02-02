/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotPrimitive.h                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Robot API primitive object.      //
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


#ifndef _acpRobotPrimitive_H_
#define _acpRobotPrimitive_H_

#include "acpObject.h"
#include "aTEAvm.h"
#include "acpTag_PRIMITIVE.h"

#define aROBOT_BEHAVIOR_PID 0

class acpHTMLPage;
class acpRobotInternal;
class acpRobotBehavior;
class acpRobotBehaviorList;
class acpRobotPacketMessage;
class acpRobotUserProperty;


class acpRobotPrimitive :
  public acpObject
{
  public:
	  			acpRobotPrimitive(
	  			  const char* pName,
	  			  acpRobotInternal* pRobotInternal,
	  			  const bool bHasAPICup = false,
	  			  const bool bHasStemCup = false);

				acpRobotPrimitive(
				  acpRobotInternal* pcRobotInternal,
				  acpTag_PRIMITIVE* pPrim);

    virtual 			~acpRobotPrimitive();

    aErr			writeToStream(const aStreamRef stream) const
    				  { return aErrNone; }

    const unsigned long		getCodeSize()
  			  	  { return m_ulCodeSize; }
  
    void			setSlot(unsigned char nSlot)
  			  	  { m_nSlot = nSlot; }
    const unsigned int		getSlot()
  			  	  { return m_nSlot; }

    aPROPERTY_FLAGS		getPropertyType(
    				  const int nPropertyIndex);

    void			validate();

    virtual void		execute(acpRobotBehavior* pBehavior);
    
    virtual const char*		getBasicPrimitiveHTML();
    virtual const char*		getParamHTML();

    virtual acpRobotBehavior*	factoryBehavior(
    				  const char* pBehaviorName,
    				  acpRobotPrimitive* pPrimitive,
    				  const int nID,
    				  acpRobotInternal* pRobotInternal,
    				  acpRobotBehaviorList* pParent = NULL);

    virtual acpRobotBehavior*	factoryBehavior(acpHTMLPage& page);

    bool			hasStemCup()
    				  { return m_bHasStemCup; }

  protected: 

    int				addPrimitiveProperty(
    				  acpRobotUserProperty* pProperty,
    				  acpValue* pDefault);

    static aErr			m_vmExitProc(
				  const aVMExit eExitCode,
				  const char* returnData,
				  const unsigned char returnDataSize,
				  const aTEAProcessID pid,
				  const void* ref);

    void			processMessage(
    				  acpRobotPacketMessage* pPacketMessage);

    acpRobotInternal*		m_pcRobotInternal;  
    bool			m_bHasAPICup;
    bool			m_bHasStemCup;

    unsigned char		m_nModule;
    char*			m_pCode;
    unsigned long		m_ulCodeSize;
    unsigned char		m_nSlot;
    unsigned char		m_cPID;
    
    int				m_nUserPropCt;
    int				m_nInputSize;
    char*			m_pAPICUP;
    unsigned long		m_ulAPICUPSize;
    
    int				m_nLinkID;


  friend class acpRobotInternal;
};

#endif // _acpRobotPrimitive_H_
