/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_PATCHBAY.cpp	 	 		           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Package tag class.				   //
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

#include "acpTag_PATCHBAY.h"

#include "acpShort.h"



/////////////////////////////////////////////////////////////////////
// PATCHBAY = 0x0006
/////////////////////////////////////////////////////////////////////

acpTag_PATCHBAY::~acpTag_PATCHBAY()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_PATCHBAY::setData(
  acpList<acpConnection>& connections)
{
  acpConnection* pConnection;
  while ((pConnection = connections.removeHead()))
    m_connections.add(pConnection);
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_PATCHBAY::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_PATCHBAY* pPatchBay = new acpTag_PATCHBAY();

  // get the number of connections
  acpShort numConnections = acpShort(stream);

  // get each connection
  for (int i = 0; i < numConnections; i++)
    pPatchBay->m_connections.add(new acpConnection(stream));

  return pPatchBay;
}


/////////////////////////////////////////////////////////////////////

void acpTag_PATCHBAY::writeData(
  aStreamRef stream
) const
{
  // the number of connections
  acpShort numConnections((short)m_connections.length());
  numConnections.writeToStream(stream);

  // the connections
  aLISTITERATE(acpConnection, m_connections, pConnection) 
    pConnection->writeToStream(stream);
}
