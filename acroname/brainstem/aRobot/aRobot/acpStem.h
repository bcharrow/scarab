/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpStem.h                                                 //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of C++ wrapper class for Stem.          //
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


#ifndef _acpStem_H_
#define _acpStem_H_

#include "aIO.h"
#include "aUI.h"
#include "aStem.h"

class acpStem
{
  public:
	  		acpStem(
	  		  aIOLib ioRef,
	  		  aUILib uiRef,
	  		  const char* pSettingFileName,
	  		  aHeartbeatCallback hbProc,
	  		  const int nID,
	  		  void* pAppRef,
	  		  const char* sName = "");
    virtual		~acpStem();

    void*		getAppRef()
			{ return m_pAppRef; }
    int			getID() const
			{ return m_nID; }

    aLIBRETURN		GetData(
			  const aPacketRef packetRef,
			  unsigned char* pModule,
			  unsigned char* pLength,
			  char* data,
			  aErr* pErr);

    aLIBRETURN		GetPacket(
			  aPacketFilter filterProc,
			  void* filterRef,
			  unsigned long nMSTimeout,
			  aPacketRef* pPacketRef,
			  aErr* pErr);

    aLIBRETURN		CreatePacket(
			  unsigned char module,
			  unsigned char length,
			  char* data,
			  aPacketRef* pPacketRef,
			  aErr* pErr);

    aLIBRETURN		SendPacket(
			  const aPacketRef packetRef,
			  aErr* pErr);

    aLIBRETURN		DestroyPacket(
			  const aPacketRef packetRef,
			  aErr* pErr);

    aLIBRETURN		DebugLine(
			  const char* data,
			  aErr* pErr);

  private:
    aIOLib			m_ioRef;
    aStemLib			m_stemRef;
    aSettingFileRef		m_settings;
    aStreamRef			m_linkStream;
    void*			m_pAppRef;
    int				m_nID;
};

#endif // _acpStem_H_
