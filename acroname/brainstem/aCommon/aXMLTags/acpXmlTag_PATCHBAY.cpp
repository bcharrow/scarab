/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_PATCHBAY.cpp	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of XML tag class.                   //
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

#include "acpXmlTag_PATCHBAY.h"

#include "acpTag_PATCHBAY.h"
#include "acpConnection.h"
#include "acpPkgImport.h"
#include "acpSimulation.h"



/////////////////////////////////////////////////////////////////////

PATCHBAY::~PATCHBAY()
{
  while (!m_connections.isEmpty())
    m_connections.removeHead();
}


/////////////////////////////////////////////////////////////////////

bool PATCHBAY::addTag(
  acpPkgXML* pTag
)
{
  const char* pTagName = pTag->tagName();
  if (!aStringCompare(pTagName, "CONNECTION")) {
    m_connections.add((CONNECTION*)pTag);
    return true;
  }

  return acpPkgXML::addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

void PATCHBAY::traverse()
{
  acpTag_PATCHBAY* pPatchBayTag = new acpTag_PATCHBAY();
  CONNECTION* pConnectionDesc;
  acpListIterator<CONNECTION> connectionDescs(m_connections);
  acpList<acpConnection> connections;
  
  // walk the list of connection descriptions and build the 
  // connections based on referenced id
  while ((pConnectionDesc = connectionDescs.next())) {

    // resolve the input side
    if (!pConnectionDesc->m_pInput) {
      pConnectionDesc->tagError("INPUT required");
      return;	
    }
    if (!pConnectionDesc->m_pInput->m_pName) {
      pConnectionDesc->m_pInput->tagError("INPUT requires a NAME");
      return;
    }
    aShort inputID = -1;
    acpPackageTag* pInputTag = 
    	m_pImporter->getNamedTag(pConnectionDesc->m_pInput->m_pName->m_name);
    if (pInputTag)
      inputID = pInputTag->getID();
    if (inputID == -1) {
      acpString msg("INPUT references unknown object: '");
      msg += pConnectionDesc->m_pInput->m_pName->m_name;
      msg += "'";
      pConnectionDesc->m_pInput->tagError(msg);
      return;
    }
    if (!pConnectionDesc->m_pInput->m_pPort) {
      pConnectionDesc->m_pInput->tagError("INPUT requires a PORT");
      return;
    }
    acpString inputName = pConnectionDesc->m_pInput->m_pPort->m_name;

    // resolve the output side
    if (pConnectionDesc->m_pOutput 
    	&& pConnectionDesc->m_pViewControl) {
      pConnectionDesc->tagError("OUTPUT or VIEWCONTROL cannot both be set");
      return;
    }

    aShort outputID = -1;
    acpString outputName;
    if (pConnectionDesc->m_pViewControl) {	
      outputID = aCONTROLPSEUDOID;
      outputName = pConnectionDesc->m_pViewControl->m_pName->m_name;
    } else {
      if (!pConnectionDesc->m_pOutput) {
      	pConnectionDesc->tagError("OUTPUT required");
        return;
      }	
      if (!pConnectionDesc->m_pOutput->m_pName) {
        pConnectionDesc->m_pOutput->tagError("OUTPUT requires a NAME");
        return;
      }
      acpPackageTag* pOutputTag = 
     	m_pImporter->getNamedTag(pConnectionDesc->m_pOutput->m_pName->m_name);
      if (pOutputTag)
        outputID = pOutputTag->getID();
    }
    if (outputID == -1) {
      acpString msg("OUTPUT references unknown object: '");
      msg += pConnectionDesc->m_pOutput->m_pName->m_name;
      msg += "'";
      pConnectionDesc->m_pOutput->tagError(msg);
    }
    if (outputID > 0) {
    	
      if (!pConnectionDesc->m_pOutput->m_pPort) {
        pConnectionDesc->m_pOutput->tagError("OUTPUT requires a PORT");
        return;
      }
      outputName = pConnectionDesc->m_pOutput->m_pPort->m_name;
    }

    connections.add(new acpConnection(inputID, inputName,
    				      outputID, outputName));
  }
  pPatchBayTag->setData(connections);
  m_pImporter->addTag(this, pPatchBayTag);
}
