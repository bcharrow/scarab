/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotVMManager.h                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definitions of the Robot API VM Manager.           //
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

#ifndef _acpRobotVMManager_H_
#define _acpRobotVMManager_H_

#include "aTEAvm.h"
#include "aRobotProperties.h"

class acpRobotInternal;



/////////////////////////////////////////////////////////////////////

class acpRobotVMManager
{
  typedef enum eVMDataType {
    eTxData = 0,
    eRxData,
    eObjData,
    eAbortData
  } eVMDataType;

  public:
				acpRobotVMManager(
				  acpRobotInternal* pRobotInternal,
				  const int nPID);
    virtual			~acpRobotVMManager();

    void			launchProcess(
				  const int nLinkID,
				  char* pCUP,
				  unsigned long ulSize,
				  char* pcInputBuff,
				  aTEAVMExitProc exitProc,
				  void* pRef);
    void			killProcess();

    void			withinThreadLaunch(
				  const int nLinkID,
				  char* pCUP,
				  unsigned long ulSize,
				  char* pcInputBuff,
				  aTEAVMExitProc exitProc,
				  void* pRef);
    void			withinThreadKill();

    bool			isBusy()
				{ return (m_bActive); }
    void			setNotActive()
				{ m_bActive = false; }
    void			setStemActiveFlag(
				  const bool bStemActive)
				{ m_bStemActive = bStemActive; }
    bool			getStemActiveFlag()
				{ return m_bStemActive; }

    void			filterPacket(
				  const unsigned char address,
				  const unsigned char length,
				  char* data);
    void			checkTimeout(
				  const unsigned long now);
    void			performTx();
    void			performRx(
				  aTEAProcessID pid,
				  aTEAVMIOPortIOCallback portIO);

    char*			getPropDataPtr() const
				{ return m_pPropData; }
    void			setPropDataPtr(
				  char* pPropData,
				  const int nSize)
				{ m_pPropData = pPropData;
				  m_nPropDataSize = nSize; }

    char			getTxLength() const
				{ return m_cTxPacketLength; }
    void			setTxLength(
				  const char cLen)
				{ m_cTxPacketLength = cLen; }

    void			addAbortByte(
				  const char cByte);
    void			SetAbortModule(
				  const char cByte)
				{ m_cAbortModule = cByte; }
    char			getAbortMsgLength() const
				{ return m_cAbortMsgLen; }
    void			setAbortMsgLength(
				  const char cLen)
				{ m_cAbortMsgLen = cLen; }

    void			setTimeout(
				  const int nTimeout)
				{ m_nTimeout = nTimeout; }
    int				getTimeout() const
				{ return m_nTimeout; }

    void			setReplyByte(
				  const char cReplyByte)
				{ m_cReplyByte = cReplyByte; }
    char			getReplyByte() const
				{ return m_cReplyByte; }

    void			setModule(
				  const char cModule)
				{ m_cModule = cModule; }

    char			getRxLength() const
				{ return m_cRxPacketLength; }
    void			setRxLength(
				  const char cLen)
				{ m_cRxPacketLength = cLen; }
    char*			getRxDataPtr(
				  const int i)
				{ return &m_RxData[i]; }


    void			readData(
				  const int dataSize,
				  char* data,
				  const int index,
				  const eVMDataType eType,
				  aErr* pErr);

    void			writeData(
				  const int dataSize,
				  char* data,
				  const int index,
				  const eVMDataType eType,
				  aErr* pErr);


    static aErr			m_vmFetchProc(
                                  const aTEAProcessID nPID,
                                  const tADDRESS nOffset,
                                  char* pData,
                                  const tADDRESS nDataSize,
                                  void* ref);
    static aErr			m_vmPopCmdProc(
				  aTEAProcessID pid,
				  char* data,
				  tBYTE dataSize,
				  void* ref);
    static aErr 		m_vmPortProc(
    				  aTEAProcessID pid,
		       		  tADDRESS port,
			     	  aBool bRead,
			     	  char* data,
			     	  tBYTE dataSize,
		                  aTEAVMIOPortIOCallback portIO,
			     	  void* ref);

  private:

    void			resetPacket();

    acpRobotInternal*		m_pcRobotInternal;
    aTEAVMIOPortIOCallback	m_portIO;

    int				m_nPID;
    char*			m_pCUP;
    unsigned long		m_ulSize;
    bool			m_bActive;
    bool			m_bHasStemCup;
    bool			m_bStemActive;

    char			m_TxData[aSTEMMAXPACKETBYTES];
    char			m_RxData[aSTEMMAXPACKETBYTES];
    unsigned char		m_cTxPacketLength;
    unsigned char		m_cRxPacketLength;

    unsigned char		m_cModule;
    unsigned char		m_cReplyByte;
    int				m_nTimeout;
    int				m_nLinkID;

    bool			m_bWaiting;
    unsigned long		m_nWaitStartTime;

    char			m_AbortMsg[aSTEMMAXPACKETBYTES];
    char			m_cAbortModule;
    unsigned char		m_cAbortMsgLen;
    
    char*			m_pPropData;
    int				m_nPropDataSize;
};

#endif // _acpRobotVMManager_H_
