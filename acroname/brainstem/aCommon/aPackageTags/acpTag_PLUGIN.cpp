/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_PLUGIN.cpp	 	 		           //
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

#include "acpTag_PLUGIN.h"

#include "acpShort.h"




/////////////////////////////////////////////////////////////////////
// PLUGIN = 0x0004
/////////////////////////////////////////////////////////////////////

acpTag_PLUGIN::~acpTag_PLUGIN()
{ 
}


/////////////////////////////////////////////////////////////////////

void acpTag_PLUGIN::setData(
  const char* pPluginName,
  const aShort nOwnerID,
  const char* pName,
  const acpVec3& translation,
  const acpList<acpParameter>& params
)
{
  setOwnerID(nOwnerID);

  m_pluginName = pPluginName;
  m_name = pName;
  m_translation = translation;

  // build up a copy of the parameter list passed in
  aLISTITERATE(acpParameter, params, pParameter) {
    acpParameter* pParamCopy = new acpParameter(pParameter);
    m_params.add(pParamCopy);
  }
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_PLUGIN::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_PLUGIN* pPlugin = new acpTag_PLUGIN();

  pPlugin->setOwnerID(acpShort(stream));
  pPlugin->m_pluginName = acpStringIO(stream);
  pPlugin->m_name = acpStringIO(stream);
  pPlugin->m_translation = acpVec3(stream);

  acpShort numParams(stream);
  for (int i = 0; i < numParams; i++)
    pPlugin->m_params.add(acpParameter::factoryFromStream(stream));

  return pPlugin;
}


/////////////////////////////////////////////////////////////////////

void acpTag_PLUGIN::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);
  m_pluginName.writeToStream(stream);
  m_name.writeToStream(stream);
  m_translation.writeToStream(stream);

  acpShort numParams((short)m_params.length());
  numParams.writeToStream(stream);
  aLISTITERATE(acpParameter, m_params, pParameter) {
    pParameter->writeToStream(stream);
  }
}
