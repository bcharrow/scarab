/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaPrimitive.h                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API primitive object.     //
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


#ifndef _acpGarciaPrimitive_H_
#define _acpGarciaPrimitive_H_

#include "acpObject.h"

#define aGARCIA_BEHAVIOR_PID 0

class acpGarciaInternal;
class acpBehavior;
class acpBehaviorList;
class acpHTMLPage;
class acpPacketMessage;

// this number is for the unique-id and primitive-name
// properties added by the behavior

class acpGarciaPrimitive :
  public acpObject
{
  public:
	  			acpGarciaPrimitive(
	  			  const char* pName,
	  			  acpGarciaInternal* pGarciaInternal);
    virtual 			~acpGarciaPrimitive();

    aErr			writeToStream(const aStreamRef stream) const
    				  { return aErrNone; }

    const unsigned int		getCodeSize()
  			  	  { return m_nCodeSize; }
  
    void			setSlot(unsigned char nSlot)
  			  	  { m_nSlot = nSlot; }
    const unsigned int		getSlot()
  			  	  { return m_nSlot; }

    aPROPERTY_FLAGS		getPropertyType(
    				  const int nPropertyIndex);

    void			validate();

    virtual void		execute(acpBehavior* pBehavior) = 0;
    
    virtual const char*		getBasicPrimitiveHTML();

    virtual const char*		getParamHTML() = 0;

    virtual acpBehavior*	factoryBehavior(
    				  const char* pBehaviorName,
    				  acpGarciaPrimitive* pPrimitive,
    				  const int nID,
    				  acpGarciaInternal* pGarciaInternal,
    				  acpBehaviorList* pParent = NULL);

    virtual acpBehavior*	factoryBehavior(acpHTMLPage& page);

    bool			isImmediate()
    				  { return m_nCodeSize == 0; }

  protected: 
    void			processMessage(
    				  acpPacketMessage* pPacketMessage);

    acpGarciaInternal*		m_pcGarciaInternal;  
    unsigned char		m_nModule;
    char*			m_pCode;
    unsigned int		m_nCodeSize;
    unsigned char		m_nSlot;
};

#endif // _acpGarciaPrimitive_H_
