/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotVMManager.cpp                                     //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of the Robot API                    //
//              user defined property.                             //
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

#include "aCmd.tea"
#include "aAPIPorts.tea"

#include "aUtil.h"
#include "aStemCore.h"

#include "acpRobotVMLaunchMessage.h"
#include "acpRobotVMManager.h"
#include "acpRobotInternal.h"





/////////////////////////////////////////////////////////////////////

acpRobotVMManager::acpRobotVMManager (
  acpRobotInternal* pRobotInternal,
  const int nPID
) :
  m_pcRobotInternal(pRobotInternal),
  m_portIO(NULL),
  m_nPID(nPID),
  m_pCUP(NULL),
  m_ulSize(0),
  m_bActive(false),
  m_bHasStemCup(false),
  m_bStemActive(false),
  m_nLinkID(aROBOT_APITEA_NOTFOUND),
  m_pPropData(NULL),
  m_nPropDataSize(0)
{
  resetPacket();
}

acpRobotVMManager::~acpRobotVMManager ()
{
}



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager resetPacket
//

void acpRobotVMManager::resetPacket()
{
  aBZero(m_TxData, aSTEMMAXPACKETBYTES);
  m_cTxPacketLength = 0;
  aBZero(m_RxData, aSTEMMAXPACKETBYTES);
  m_cRxPacketLength = 0;

  m_cReplyByte = 0;
  m_nTimeout = 0;
  m_cModule = 0;

  m_bWaiting = false;
  m_nWaitStartTime = 0;

  aBZero(m_AbortMsg, aSTEMMAXPACKETBYTES);
  m_cAbortMsgLen = 0;
}



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager addAbortByte
//

void acpRobotVMManager::addAbortByte(
  const char cByte)
{
  if (m_cAbortMsgLen < aSTEMMAXPACKETBYTES)
    m_AbortMsg[m_cAbortMsgLen++] = cByte;
}



/////////////////////////////////////////////////////////////////////
// execute API TEA program
//

void acpRobotVMManager::launchProcess(
  const int nLinkID,
  char* pCUP,
  unsigned long ulSize,
  char* pcInputBuff,
  aTEAVMExitProc exitProc,
  void* pRef
)
{
  aAssert(!m_bActive);
  
  acpRobotVMLaunchMessage* pMessage = 
    new acpRobotVMLaunchMessage(
      m_pcRobotInternal,
      nLinkID,
      m_nPID,
      pCUP,
      ulSize,
      pcInputBuff,
      exitProc,
      pRef);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcRobotInternal->m_pcThread->sendMessage(pMessage, false);
}



/////////////////////////////////////////////////////////////////////
// execute API TEA program
//

void acpRobotVMManager::killProcess()
{
  acpRobotVMKillMessage* pMessage = 
    new acpRobotVMKillMessage(
      m_pcRobotInternal,
      m_nPID);

  // This is a non-async call (the second parameter is false)
  // which causes this thread to block until the message is
  // completed.
  aAssert(pMessage);
  m_pcRobotInternal->m_pcThread->sendMessage(pMessage, false);
}



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager withinThreadLaunch method
//

void acpRobotVMManager::withinThreadLaunch(
  const int nLinkID,
  char* pCUP,
  unsigned long ulSize,
  char* pcInputBuff,
  aTEAVMExitProc exitProc,
  void* pRef
)
{
  aErr err;
  aAssert(m_pcRobotInternal->m_pcThread->isThreadContext());

  resetPacket();
  m_pCUP = pCUP;
  m_ulSize = ulSize;
  m_nLinkID = nLinkID;
  
  if (m_pCUP) {

    aTEAvmLaunchBlock lb;
    lb.popCmdProc = m_vmPopCmdProc;
    lb.popCmdRef = (void*)m_pcRobotInternal;
    lb.fetchProc = m_vmFetchProc;
    lb.fetchRef = (void*)m_pcRobotInternal;
    lb.data = pcInputBuff;
    lb.dataSize = m_pCUP[5];
    lb.retValSize = m_pCUP[4];
    lb.codeSize = (unsigned short)m_ulSize;
    lb.exitProc = exitProc;
    lb.exitRef = pRef;
    lb.flags = fTEAvmSetPID | fTEAvmRobotAPI;
    lb.pid = (unsigned char)m_nPID;

    aTEAvm_Launch(m_pcRobotInternal->m_vmRef, &lb, &err);
    m_bActive = true;
  }

} // acpRobotVMManager withinThreadLaunch method



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager withinThreadKill method
//

void acpRobotVMManager::withinThreadKill()
{
  aErr err;
  aAssert(m_pcRobotInternal->m_pcThread->isThreadContext());

  // active flag means we are running an API TEA program
  // so killing this behavior will cause all behaviors to die
  // as their finish methods are executed
  if (m_bActive) {
  
    // API program must set/clear Stem active (running) flag
    // and assign an abort message so if a Stem program is
    // running, then we kill it by sending the abort message
    if (m_bStemActive && m_cAbortMsgLen) {
      aPacketRef packet;
      m_pcRobotInternal->
        m_pStems[m_nLinkID]->
          CreatePacket(m_cAbortModule,
		     m_cAbortMsgLen,
		     m_AbortMsg,
		     &packet,
		     &err);
      aAssert(err == aErrNone);
      m_pcRobotInternal->m_pStems[m_nLinkID]->SendPacket(packet, &err);
      aAssert(err == aErrNone);

    } else {
      aTEAvm_Kill(m_pcRobotInternal->m_vmRef, (unsigned char)m_nPID, &err);
      m_bActive = false;
    }
  }
    
} // acpRobotVMManager withinThreadKill method



/////////////////////////////////////////////////////////////////////
// compare received packet with what we need
//
void acpRobotVMManager::filterPacket(
  const unsigned char address,
  const unsigned char length,
  char* data
)
{
  if (!m_bWaiting) return;

  unsigned char cmd = (unsigned char)data[0];

  // see if data matches what we need
  if ((m_cModule == address) &&
      (m_cReplyByte == cmd)) {

    // grab the data
    m_cRxPacketLength = length;
    aMemCopy(m_RxData, data, length);

    // but clear the wait info
    m_bWaiting = false;
    m_nTimeout = 0;
    m_nWaitStartTime = 0;

    // let VM resume
    m_portIO(
      m_pcRobotInternal->m_vmRef,
      (unsigned char)m_nPID,
      kTEAvmClearHalt, NULL, 0);
  }
}



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager checkTimeout
//
void acpRobotVMManager::checkTimeout(
  const unsigned long now
)
{
  // do nothing if not waiting
  // or null timeout time is specified
  if (!m_bWaiting) return;
  if (!m_nTimeout) return;

  if (now > m_nTimeout + m_nWaitStartTime) {

    // clear the data and wait info
    resetPacket();

    // let VM resume
    m_portIO(
      m_pcRobotInternal->m_vmRef,
      (unsigned char)m_nPID,
      kTEAvmClearHalt, NULL, 0);
  }
}


/////////////////////////////////////////////////////////////////////
// acpRobotVMManager performTx
//
void acpRobotVMManager::performTx()
{
  aErr err;
  aPacketRef packet;

  m_pcRobotInternal->
    m_pStems[m_nLinkID]->
      CreatePacket(
        m_cModule,
        m_cTxPacketLength,
        m_TxData,
        &packet,
        &err);
  aAssert(err == aErrNone);

  m_pcRobotInternal->m_pStems[m_nLinkID]->SendPacket(packet, &err);
  aAssert(err == aErrNone);
}



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager performRx
//
void acpRobotVMManager::performRx(
  aTEAProcessID pid,
  aTEAVMIOPortIOCallback portIO
)
{
  // reset RX packet
  aBZero(m_RxData, aSTEMMAXPACKETBYTES);
  m_cRxPacketLength = 0;

  // indicate we are waiting for data
  m_portIO = portIO;
  m_bWaiting = true;

  // initialize start time for timeout check
  aIO_GetMSTicks(
    m_pcRobotInternal->m_ioRef,
    &m_nWaitStartTime,
    NULL);

  // then halt the VM
  portIO(m_pcRobotInternal->m_vmRef, pid, kTEAvmSetHalt, NULL, 0);
}



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager readData
//
void acpRobotVMManager::readData(
  const int dataSize,
  char* data,
  const int index,
  const eVMDataType eType,
  aErr* pErr
)
{
  aErr err = aErrNone;
  int nsize = 0;
  char* pVMData = NULL;

  switch (eType) {
    case eTxData:
    {
      pVMData = m_TxData;
      nsize = aAPIPortTXBufferSize;
      break;
    }
    case eRxData:
    {
      pVMData = m_RxData;
      nsize = aAPIPortRXBufferSize;
      break;
    }
    case eObjData:
    {
      pVMData = m_pPropData;
      nsize = m_nPropDataSize;
      break;
    }
    case eAbortData:
    {
      pVMData = m_AbortMsg;
      nsize = aAPIPortAbortMsgSize;
      break;
    }
  }

  // bounds check on the data index
  if ((index < 0) 
    || (dataSize == 2 && (index >= (nsize - 1)))
    || (dataSize == 1 && (index >= nsize))) {
    err = aErrIO;
  }

  if (pVMData && (err == aErrNone)) {
    data[0] = pVMData[index];
    if (dataSize == 2) {
      // data[1] = pVMData[index + 1];
      // watch out for byte ordering!!!
      data[0] = pVMData[index + 1];
      data[1] = pVMData[index];
    }
  }
  
  if (pErr)
    *pErr = err;
}



/////////////////////////////////////////////////////////////////////
// acpRobotVMManager writeData
//
void acpRobotVMManager::writeData(
  const int dataSize,
  char* data,
  const int index,
  const eVMDataType eType,
  aErr* pErr
)
{
  aErr err = aErrNone;
  int nsize = 0;
  char* pVMData = NULL;

  switch (eType) {
    case eTxData:
    {
      pVMData = m_TxData;
      nsize = aAPIPortTXBufferSize;
      break;
    }
    case eRxData:
    {
      pVMData = m_RxData;
      nsize = aAPIPortRXBufferSize;
      break;
    }
    case eObjData:
    {
      pVMData = m_pPropData;
      nsize = m_nPropDataSize;
      break;
    }
    case eAbortData:
    {
      pVMData = m_AbortMsg;
      nsize = aAPIPortAbortMsgSize;
      break;
    }
  }

  // bounds check on the data index
  if ((index < 0) 
    || (dataSize == 2 && (index >= (nsize - 1)))
    || (dataSize == 1 && (index >= nsize))) {
    err = aErrIO;
  }

  if (pVMData) {
    pVMData[index] = data[0];
    if (dataSize == 2) {
      pVMData[index + 1] = data[1];
    }
  }

  if (pErr)
    *pErr = err;
}



/////////////////////////////////////////////////////////////////////
// virtual machine fetch (from memory) proc
//

aErr acpRobotVMManager::m_vmFetchProc(
  const aTEAProcessID nPID,
  const tADDRESS nOffset,
  char* pData,
  const tADDRESS nDataSize,
  void* ref
) 
{
  char* pCUP = NULL;

  acpRobotInternal* pBot =   
    (acpRobotInternal*)ref;
  acpRobotVMManager* pVMM = pBot->m_pcVMM[nPID];

  aAssert(pVMM);
  
  pCUP = pVMM->m_pCUP;
  aAssert(pCUP);

  aMemCopy(pData,
	   &pCUP[nOffset],
	   nDataSize);

  return aErrNone;
}



/////////////////////////////////////////////////////////////////////
// pop command proc
//

aErr acpRobotVMManager::m_vmPopCmdProc(
  aTEAProcessID pid,
  char* data,
  tBYTE dataSize,
  void* ref)
{
  return aErrNone;
}


/////////////////////////////////////////////////////////////////////
// port command proc
//

aErr acpRobotVMManager::m_vmPortProc(
  aTEAProcessID pid,
  tADDRESS port,
  aBool bRead,
  char* data,
  tBYTE dataSize,
  aTEAVMIOPortIOCallback portIO,
  void* ref
) 
{
  aErr vmErr = aErrNone;

  acpRobotInternal* pBot = 
    (acpRobotInternal*)ref;
  acpRobotVMManager* pVMM = pBot->m_pcVMM[pid];

  switch(port) {

    case aAPIPortBeginTX:
    {
      // serial TX control port a write-only byte
      if (bRead) {
        vmErr = aErrIO;
      } else {
        if (dataSize != 1) {
          vmErr = aErrIO;
        } else {
          // record which module will be getting data from us
          pVMM->setModule(*data);
          pVMM->performTx();
        }
      }
      break;
    }

    case aAPIPortBeginRX:
    {
      // serial RX control port a write-only byte
      if (bRead) {
        vmErr = aErrIO;
      } else {
        if (dataSize != 1) {
          vmErr = aErrIO;
        } else {
          // record which module will be sending us data
          pVMM->setModule(*data);
          pVMM->performRx(pid, portIO);
        }
      }
      break;
    }

    case aAPIPortTXLength:
    {
      // TX length is a byte
      if (dataSize != 1) {
        vmErr = aErrIO;
      } else {
        if (bRead) {
          *data = pVMM->getTxLength();
        } else {
          pVMM->setTxLength(*data);
        }
      }
      break;
    }

    case aAPIPortRXLength:
    {
      // RX length is a byte
      if (dataSize != 1) {
        vmErr = aErrIO;
      } else {
        if (bRead) {
          *data = pVMM->getRxLength();
        } else {
          pVMM->setRxLength(*data);
        }
      }
      break;
    }

    case aAPIPortRXTimeout:
    {
      // timeout is a write-only short
      if (bRead) {
        vmErr = aErrIO;
      } else {
        if (dataSize != 2) {
          vmErr = aErrIO;
        } else {
          pVMM->setTimeout(aUtil_RetrieveShort(data));
        }
      }
      break;
    }

    case aAPIPortRXFilterByte:
    {
      // reply byte is a write-only byte
      if (bRead) {
        vmErr = aErrIO;
      } else {
        if (dataSize != 1) {
          vmErr = aErrIO;
        } else {
          pVMM->setReplyByte(*data);
        }
      }
      break;
    }

    case aAPIPortBeginTXRX:
    {
      // TXRX is a write-only short
      if (bRead) {
        vmErr = aErrIO;
      } else {
        if (dataSize != 2) {
          vmErr = aErrIO;
        } else {
          char txmodule = data[0];
          char rxmodule = data[1];
          pVMM->setModule(txmodule);
          pVMM->performTx();
          pVMM->setModule(rxmodule);
          pVMM->performRx(pid, portIO);
        }
      }
      break;
    }

    case aAPIPortActiveFlag:
    {
      // active flag is write-only byte
      if (dataSize != 1) {
        vmErr = aErrIO;
      } else {
        if (bRead) {
          vmErr = aErrIO;
        } else {
          pVMM->setStemActiveFlag((bool)*data); 
        }
      }
      break;
    }

    case aAPIPortAbortModule:
    {
      // abort message module port
      if (bRead) {
        vmErr = aErrIO;
      } else {
        if (dataSize != 1) {
          vmErr = aErrIO;
        } else {
          pVMM->SetAbortModule(*data);
        }
      }
      break;
    }

    case aAPIPortAbortMsgAdd:
    {
      // abort message data port
      if (bRead) {
        vmErr = aErrIO;
      } else {
        if (dataSize == 2) {
          pVMM->addAbortByte(data[0]);
          pVMM->addAbortByte(data[1]);
        } else {
          pVMM->addAbortByte(data[0]);
        }
      }
      break;
    }

    case aAPIPortAbortMsgLength:
    {
      // Abort msg length is a byte
      if (dataSize != 1) {
        vmErr = aErrIO;
      } else {
        if (bRead) {
          *data = pVMM->getAbortMsgLength();
        } else {
          pVMM->setAbortMsgLength(*data);
        }
      }
      break;
    }

    default:
    {
      if ((port >= aAPIPortTXBuffer) &&
          (port <= aAPIPortTXBuffer + aAPIPortTXBufferSize)) {
        // TX data access
        int index = port - aAPIPortTXBuffer;
        if (bRead) {
          pVMM->readData(dataSize, data, index, eTxData, &vmErr);
        } else {
          pVMM->writeData(dataSize, data, index, eTxData, &vmErr);
        }
      } else if ((port >= aAPIPortRXBuffer) &&
                 (port <= aAPIPortRXBuffer + aAPIPortRXBufferSize)) {
        // RX data access
        int index = port - aAPIPortRXBuffer;
        if (bRead) {
          pVMM->readData(dataSize, data, index, eRxData, &vmErr);
        } else {
          pVMM->writeData(dataSize, data, index, eRxData, &vmErr);
        }
      } else if ((port >= aAPIPortAbortMsg) &&
                 (port <= aAPIPortAbortMsg + aAPIPortAbortMsgSize)) {
        // abort message data access
        int index = port - aAPIPortAbortMsg;
        if (bRead) {
          pVMM->readData(dataSize, data, index, eAbortData, &vmErr);
        } else {
          pVMM->writeData(dataSize, data, index, eAbortData, &vmErr);
        }
      } else if ((port >= aAPIPortObjBuffer) &&
                 (port <= aAPIPortObjBuffer + aAPIPortObjBufferSize) &&
                 (pid == aROBOT_APITEA_PROPPROCID)) {
        // user object data access
        int index = port - aAPIPortObjBuffer;
        if (bRead) {
          pVMM->readData(dataSize, data, index, eObjData, &vmErr);
        } else {
          pVMM->writeData(dataSize, data, index, eObjData, &vmErr);
        }
      } else {
        vmErr = aErrIO;
      }
      break;
    }
  }

  return vmErr;
}
