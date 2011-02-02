/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotClient.h                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotClient client.             //
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

#ifndef _acpRobotClient_H_
#define _acpRobotClient_H_

#include "aIO.h"
#include "acpString.h"


class acpRobotClient
{
  public:
  				acpRobotClient();
    virtual			~acpRobotClient();

  protected:
    aIOLib			m_ioRef;

    // TCP/IP interaction
    bool			m_bConnected;
    unsigned long		m_nTimeout;
    unsigned long		m_ipaddress;
    unsigned short		m_ipport;
    aStreamRef			m_stream;
    aStreamRef			m_bufferStream;

    int                         connect(
				  const int nAddress,
				  const int nPort,
				  const int nTimeout);

    void                        reset();

    // enumerator callback type for properties of an object
    typedef void (*robotPropEnumProc)(const char* pName,
				      const int nIndex,
				      const int typeflags,
				      void* vpRef);

  public:
    void                        writeBoolProperty(
				  const int nObjectID,
				  const int nPropIndex,
				  const bool value);

    void                        writeIntProperty(
				  const int nObjectID,
				  const int nPropIndex,
				  const int value);

    void                        writeFloatProperty(
				  const int nObjectID,
				  const int nPropIndex,
				  const float value);

    void                        writeStringProperty(
				  const int nObjectID,
				  const int nPropIndex,
				  const char* pValue);

    float                       readFloatProperty(
				  const int nObjectID,
				  const int nPropIndex);

    int                         readIntProperty(
				  const int nObjectID,
				  const int nPropIndex);

    void                        enumProperties(
				  const int objectID,
				  robotPropEnumProc enumerator,
				  void* vpRef);

    void	                getObjectName(
    				  acpString* pName,
    				  const int nObjectID);

    void	                getPropertyDescription(
    				  acpString* pDescription,
    				  const int nObjectID,
    				  const int nPropIndex);

    bool			sendChar(
    				  const char data,
    				  const bool bFinal = false);
    bool			sendShort(
    				  const short data,
    				  const bool bFinal = false);
    bool			sendInt(
    				  const int data,
    				  const bool bFinal = false);
    bool			sendFloat(
    				  const float data,
    				  const bool bFinal = false);
    bool			sendString(
    				  const char* data,
    				  const bool bFinal = false);
    int				nextByte();
    short			nextShort();
    int				nextInt();
    float                       nextFloat();
    bool			nextString(
    				  acpString* pString);

  protected:
    int			        ack();
    void			bye();
    bool                        linkUp(
				  const int nMSTimeout);
};


#endif // _acpRobotClient_H_
