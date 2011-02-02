/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_INSTANCE.cpp	 	 		           //
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

#include "acpTag_INSTANCE.h"

#include "acpStringIO.h"
#include "acpShort.h"
#include "acpByte.h"


/////////////////////////////////////////////////////////////////////
// INSTANCE = 0x000C
/////////////////////////////////////////////////////////////////////


acpTag_INSTANCE::acpTag_INSTANCE() :
  acpPackageTag(aTAG_INSTANCE, (void*)1),
  m_translation()
{ 
}

/////////////////////////////////////////////////////////////////////

acpTag_INSTANCE::~acpTag_INSTANCE()
{
}


/////////////////////////////////////////////////////////////////////

void acpTag_INSTANCE::setData(
  const aShort nOwnerID,
  const char* pName,
  const acpList<acpShort>& geometry,
  const aByte nFlags,
  const acpTransform& transform
)
{
  setOwnerID(nOwnerID);
  m_name = pName;
  
  // clone the list of ids
  aLISTITERATE(acpShort, geometry, pGeomID) {
    acpShort* pID = new acpShort(*pGeomID);
    m_geometryIDs.add(pID);
  }
  m_nFlags = nFlags;
  m_translation = transform.getTranslation();
  m_rotation = transform.getRotation();
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_INSTANCE::factoryFromStream(
  aStreamRef stream
)
{
  acpTag_INSTANCE* pINSTANCE = new acpTag_INSTANCE();

  pINSTANCE->setOwnerID(acpShort(stream));
  pINSTANCE->m_name = acpStringIO(stream);
  acpShort nGeoms(stream);
  for (int i = 0; i < nGeoms; i++) {
    acpShort* pID = new acpShort(stream);
    pINSTANCE->m_geometryIDs.add(pID);
  }
  pINSTANCE->m_nFlags = acpByte(stream);
  pINSTANCE->m_translation = acpVec3(stream);
  pINSTANCE->m_rotation = acpMatrix3(stream);

  return pINSTANCE;
}


/////////////////////////////////////////////////////////////////////

void acpTag_INSTANCE::writeData(
  aStreamRef stream
) const
{
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  acpStringIO name(m_name);
  name.writeToStream(stream);

  // write the number of geometries and then the geometry IDs
  acpShort nGeoms((aShort)(m_geometryIDs.length()));
  nGeoms.writeToStream(stream);
  aLISTITERATE(acpShort, m_geometryIDs, pGeomID) {
    acpShort ID(*pGeomID);
    ID.writeToStream(stream);
  }

  acpByte flags(m_nFlags);
  flags.writeToStream(stream);

  m_translation.writeToStream(stream);
  m_rotation.writeToStream(stream);
}
