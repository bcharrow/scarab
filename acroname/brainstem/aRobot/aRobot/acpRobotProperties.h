/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotProperties.h                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definitions of the Robot API library object.       //
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

#ifndef _acpRobotProperties_H_
#define _acpRobotProperties_H_

#include "aIO.h"
#include "aTEAvm.h"

#include "acpTag_PROPERTY.h"

#include "acpRobotPackage.h"
#include "acpRobotInternal.h"
#include "acpProperty.h"





/////////////////////////////////////////////////////////////////////

class acpRobotProperty : 
  public acpProperty
{
  public:
			acpRobotProperty(
			  acpRobotInternal* pRobotInternal,
			  const char* pName,
			  aPROPERTY_FLAGS flags,
			  const char* pDescription);

    acpRobotInternal*	m_pRobotInternal;
};


/////////////////////////////////////////////////////////////////////

class acpRobotTEAProperty : 
  public acpRobotProperty
{

  public:

    typedef struct tConvBlock {
      unsigned long	ulUnit;
      char		cParamIndex;
      char		cParamSize;
      int		nConvType;
    } tConvBlock;

  public:
			acpRobotTEAProperty(
			  acpRobotInternal* pRobotInternal,
			  const char* pName,
			  aPROPERTY_FLAGS flags,
			  const char* pDescription,
			  const tConvBlock* pSetBlock,
			  const tConvBlock* pGetBlock);

    void		convertValToInput(
			  const acpValue* pValue,
			  char* pInputBuff);
    void		convertOutputToVal(
			  acpValue* pValue,
			  const char* pOutputBuff);
    virtual void	addTagData(
			  acpPackageTag* pPkg);

    char		m_cUnitType;
    char		m_cUnitCode;

    tConvBlock		m_tSetBlock;
    tConvBlock		m_tGetBlock;

    int			m_nLinkID;
};



/////////////////////////////////////////////////////////////////////

class acpRobotUserProperty : 
  public acpRobotTEAProperty
{
  public:
			acpRobotUserProperty(
			  acpRobotInternal* pcRobotInternal,
			  acpTag_PROPERTY* pProp);

    virtual		~acpRobotUserProperty();

    virtual void	addTagData(
			  acpPackageTag* pPkg);
    virtual void	setValue(
			  const acpValue* pValue);
    virtual void	getValue(
			  const acpObject* pObject,
			  acpValue* pValue);

  protected:

    static aErr		m_vmExitProc(
			  const aVMExit eExitCode,
			  const char* returnData,
			  const unsigned char returnDataSize,
			  const aTEAProcessID pid,
			  const void* ref);

  private:

    bool		m_bSet;
    bool		m_bDone;
    acpValue*		m_pValue;

    char*		m_pSetCUP;
    unsigned long	m_ulSetSize;
    char*		m_pGetCUP;
    unsigned long	m_ulGetSize;
};


/////////////////////////////////////////////////////////////////////

class acpRobotActiveProperty : public acpRobotProperty
{
  public:
  		acpRobotActiveProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotIdleProperty : public acpRobotProperty
{
  public:
  		acpRobotIdleProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	getValue(
    		  const acpObject* pObject,
		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotHeartbeatCallbackProperty : public acpRobotProperty
{
  public:
  		acpRobotHeartbeatCallbackProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotHeartbeatStatusProperty : public acpRobotProperty
{
  public:
  		acpRobotHeartbeatStatusProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotDistanceUnitsProperty : public acpRobotProperty
{
  public:
  		acpRobotDistanceUnitsProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotDistanceUnitsStringProperty : public acpRobotProperty
{
  public:
  		acpRobotDistanceUnitsStringProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotAngleUnitsProperty : public acpRobotProperty
{
  public:
  		acpRobotAngleUnitsProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotAngleUnitsStringProperty : public acpRobotProperty
{
  public:
  		acpRobotAngleUnitsStringProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotMassUnitsProperty : public acpRobotProperty
{
  public:
  		acpRobotMassUnitsProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotMassUnitsStringProperty : public acpRobotProperty
{
  public:
  		acpRobotMassUnitsStringProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotStatusProperty : public acpRobotProperty
{
  public:
  		acpRobotStatusProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotStatusStreamProperty : public acpRobotProperty
{
  public:
  		acpRobotStatusStreamProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpRobotErrorStreamProperty : public acpRobotProperty
{
  public:
  		acpRobotErrorStreamProperty (
  		  acpRobotInternal* m_pRobotInternal);

    void	setValue(
    		  const acpValue* pValue);
    void	getValue(
    		  const acpObject* pObject,
    		  acpValue* pValue);
};

#endif // _acpRobotProperties_H_
