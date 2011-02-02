/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_PLUGIN.h				           //
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

#ifndef _acpTag_PLUGIN_H_
#define _acpTag_PLUGIN_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "acpVec3.h"
#include "acpParameter.h"
#include "acpStringIO.h"



/////////////////////////////////////////////////////////////////////
// PLUGIN = 0x0004

class acpTag_PLUGIN :
  public acpPackageTag
{
  public:
  				acpTag_PLUGIN() :
  				  acpPackageTag(aTAG_PLUGIN, (void*)1)
  				  {}
  				~acpTag_PLUGIN();

    void			setData(
    				  const char* pPluginName,
    				  aShort nOwnerID,
    				  const char* pName,
    				  const acpVec3& translation,
    				  const acpList<acpParameter>& params);

    acpPackageTag*		factoryFromStream(
    				  aStreamRef stream);

  protected:
    void			writeData(
    				  aStreamRef stream) const;

  private:
    acpStringIO			m_pluginName;
    acpStringIO			m_name;
    acpVec3			m_translation;
    acpList<acpParameter>	m_params;
  
  friend class			acpSimulation;
  friend class			acpPluginWrapper;
};

#endif // _acpTag_PLUGIN_H_
