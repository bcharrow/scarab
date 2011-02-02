/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGarciaCamera.h                                         //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API camera object.        //
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

#ifndef _acpGarciaCamera_H_
#define _acpGarciaCamera_H_

#include "acpObject.h"
#include "acpValue.h"
#include "acpTextBuffer.h"
#include "acpGarciaInternal.h"


/////////////////////////////////////////////////////////////////////
// property names and associated indexes for the camera class

#define aGCP_PAN			"pan"
#define aGCP_TILT			"tilt"

#define	aGCP_CAMERA_BOOM		"camera-boom"

#define	aGCP_TILT_INDEX			3
#define	aGCP_PAN_INDEX			2



class acpGarciaInternal;
class acpGarciaCameraProperty;

class acpGarciaCamera :
  public acpObject
{
  protected:
  				acpGarciaCamera(
				  const char* pName,
  				  const unsigned char cModule,
  				  acpGarciaInternal* pcGarciaInternal);

  public:
    virtual   			~acpGarciaCamera();

    virtual unsigned char	getModule()
				  { return m_cModule; }

    virtual aErr		writeToStream(
    				  const aStreamRef stream) const;

    virtual void		getDescription(acpTextBuffer& buffer) {}

  protected:

    unsigned char		m_cModule;
    acpGarciaInternal*		m_pcGarciaInternal;

  friend class acpGarciaInternal;
  friend class acpGarciaCameraProperty;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaCameraProperty : public acpProperty
{
  public:
  				acpGarciaCameraProperty(
  				  acpGarciaCamera* pcGarciaCamera,
  				  const char* pName,
  				  aPROPERTY_FLAGS flags);

    virtual			~acpGarciaCameraProperty() {}

  protected:

    void			sendPacket (
		  		  acpGarciaCamera* pcGarciaCamera,
		  		  unsigned char cModule,
				  unsigned char cLen,
				  char* data);

    float			getServoPositionValue(unsigned char cIndex) const
				  { return m_pcGarciaCamera->m_pcGarciaInternal->getServoPositionValue(
				      (unsigned char)m_pcGarciaCamera->m_cModule, cIndex); }

  protected:

    acpGarciaCamera*		m_pcGarciaCamera;
};



/////////////////////////////////////////////////////////////////////

class acpGarciaCameraPanProperty : public acpGarciaCameraProperty
{
  public:
  				acpGarciaCameraPanProperty(
  				  acpGarciaCamera* pcGarciaCamera);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};


/////////////////////////////////////////////////////////////////////

class acpGarciaCameraTiltProperty : public acpGarciaCameraProperty
{
  public:
  				acpGarciaCameraTiltProperty(
  				  acpGarciaCamera* pcGarciaCamera);

    void			setValue (
				  const acpValue* pValue);

    void 			getValue (
				  const acpObject* pObject,
				  acpValue* pValue);
};

#endif // _acpGarciaCamera_H_
