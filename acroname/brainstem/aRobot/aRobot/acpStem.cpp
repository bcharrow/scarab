/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpStem.cpp                                               //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of C++ wrapper class for Stem.      //
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

#include "aUtil.h"
#include "aStem.h"
#include "aStreamUtil.h"

#include "acpStem.h"



/////////////////////////////////////////////////////////////////////
// acpStem constructor
//

acpStem::acpStem (
  aIOLib ioRef,
  aUILib uiRef,
  const char* pSettingFileName,
  aHeartbeatCallback hbProc,
  const int nID,
  void* pAppRef,
  const char* sName // = ""
) :
  m_ioRef(ioRef),
  m_settings(NULL),
  m_linkStream(NULL),
  m_pAppRef(pAppRef),
  m_nID(nID)
{
  aErr err = aErrNone;

  // get the aStem library reference
  aStem_GetNamedLibRef(&m_stemRef, sName, &err);
  aAssert(err == aErrNone);

  // get settings if none were provided
  if (!m_settings) {
    aSettingFile_Create(ioRef, 
			aSTEM_MAXSETTINGLEN,
			pSettingFileName,
			&m_settings, &err);
    aAssert(err == aErrNone);
  }

  // set up the stream for this stem
  aStreamUtil_CreateSettingStream(ioRef, 
  				  uiRef,
  				  m_stemRef,
  				  m_settings,
  				  &m_linkStream,
  				  &err);
  aAssert(err == aErrNone);

  // set up the heartbeat callback
  if (hbProc) {
    aStem_SetHBCallback(m_stemRef, hbProc, this, &err);
    aAssert(err == aErrNone);
  }

} // acpStem constructor


/////////////////////////////////////////////////////////////////////
// acpStem destructor
//

acpStem::~acpStem ()
{
  if (m_settings)
    aSettingFile_Destroy(m_ioRef, m_settings, NULL);
  if (m_stemRef)
    aStem_ReleaseLibRef(m_stemRef, NULL);
  
} // acpStem destructor



/////////////////////////////////////////////////////////////////////

aLIBRETURN acpStem::GetData(
  const aPacketRef packetRef,
  unsigned char* pModule,
  unsigned char* pLength,
  char* data,
  aErr* pErr
)
{
  return
    aPacket_GetData(
      m_stemRef,
      packetRef,
      pModule,
      pLength,
      data,
      pErr);
}



/////////////////////////////////////////////////////////////////////

aLIBRETURN acpStem::GetPacket(
  aPacketFilter filterProc,
  void* filterRef,
  unsigned long nMSTimeout,
  aPacketRef* pPacketRef,
  aErr* pErr
)
{
  return
    aStem_GetPacket(
      m_stemRef,
      filterProc,
      filterRef,
      nMSTimeout,
      pPacketRef,
      pErr);
}


/////////////////////////////////////////////////////////////////////

aLIBRETURN acpStem::CreatePacket(
  unsigned char module,
  unsigned char length,
  char* data,
  aPacketRef* pPacketRef,
  aErr* pErr
)
{
  return
    aPacket_Create(
      m_stemRef,
      module,
      length,
      data,
      pPacketRef,
      pErr);
}


/////////////////////////////////////////////////////////////////////

aLIBRETURN acpStem::SendPacket(
  const aPacketRef packetRef,
  aErr* pErr
)
{
  return
    aStem_SendPacket(
      m_stemRef,
      packetRef,
      pErr);
}



/////////////////////////////////////////////////////////////////////

aLIBRETURN acpStem::DestroyPacket(
  const aPacketRef packetRef,
  aErr* pErr
)
{
  return
    aPacket_Destroy(
      m_stemRef,
      packetRef,
      pErr);
}



/////////////////////////////////////////////////////////////////////

aLIBRETURN acpStem::DebugLine(
  const char* line,
  aErr* pErr
)
{
  return
    aStem_DebugLine(
      m_stemRef,
      line,
      pErr);
}
