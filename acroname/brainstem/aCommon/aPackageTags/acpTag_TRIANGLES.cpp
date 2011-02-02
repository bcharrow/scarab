/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_TRIANGLES.cpp	 	 		           //
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

#include "acpTag_TRIANGLES.h"

#include "acpException.h"
#include "acpShort.h"
#include "acpByte.h"
#include "acpInt32.h"
#include "acpFloat.h"



/////////////////////////////////////////////////////////////////////
// TRIANGLES = 0x0002
/////////////////////////////////////////////////////////////////////

acpTag_TRIANGLES::~acpTag_TRIANGLES()
{ 
  clean();
}


/////////////////////////////////////////////////////////////////////

void acpTag_TRIANGLES::setData(
  const aShort nOwnerIndex,
  const acpVec3& color,
  const aInt32 nVertices,
  const asReal* pVertices,
  const aInt32 nIndices,
  const aInt32* pIndices
)
{
  clean();
  setOwnerID(nOwnerIndex);
  m_color = color;
  m_nVertices = nVertices;
  m_pVertices = pVertices;
  m_nIndices = nIndices;
  m_pIndices = pIndices;
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpTag_TRIANGLES::factoryFromStream(
  aStreamRef stream
)
{
  int i;
  acpTag_TRIANGLES* pTriangles = new acpTag_TRIANGLES();

  // find the owner index
  acpShort ownerID(stream);
  pTriangles->setOwnerID(ownerID);

  aByte r = acpByte(stream);
  aByte g = acpByte(stream);
  aByte b = acpByte(stream);

  pTriangles->m_color = acpVec3((float)((unsigned char)r / 254.0), 
  		    	     (float)((unsigned char)g / 254.0),
  		    	     (float)((unsigned char)b / 254.0));

  pTriangles->m_nVertices = acpInt32(stream);

  // create some temporary space to read in the data
  pTriangles->m_pVertices = 
  	(asReal*)aMemAlloc(pTriangles->m_nVertices * sizeof(asReal) * 3);
  if (!pTriangles->m_pVertices)
    throw acpException(aErrMemory, 
    		       "allocating static geometry vertex data");

  asReal* pV = (asReal*)pTriangles->m_pVertices;
  for (i = 0; i < pTriangles->m_nVertices; i++) {
    *pV++ = acpFloat(stream);
    *pV++ = acpFloat(stream);
    *pV++ = acpFloat(stream);
  }

  pTriangles->m_nIndices = acpInt32(stream);

  // create some temporary space to read in the data
  pTriangles->m_pIndices = 
  	(aInt32*)aMemAlloc(pTriangles->m_nIndices * sizeof(aInt32));
  if (!pTriangles->m_pIndices)
    throw acpException(aErrMemory, 
    		       "allocating static geometry index data");

  aInt32* pI = (aInt32*)pTriangles->m_pIndices;
  for (i = 0; i < pTriangles->m_nIndices; i++, pI++) {
    *pI = acpInt32(stream);
    aAssert(*pI < pTriangles->m_nIndices);
  }

  // one normal per triangle for now
  pTriangles->m_pNormals = 
  	(asReal*)aMemAlloc(pTriangles->m_nIndices * sizeof(asReal));
  if (!pTriangles->m_pNormals)
    throw acpException(aErrMemory, 
    		       "allocating static geometry normal data");

  // compute the normals
  asReal* pN = (asReal*)pTriangles->m_pNormals;
  for (i = 0; i < pTriangles->m_nIndices; i += 3) {
    acpVec3 og(&pTriangles->m_pVertices[pTriangles->m_pIndices[i] * 3]);
    acpVec3 p1(&pTriangles->m_pVertices[pTriangles->m_pIndices[i+1] * 3]);
    acpVec3 p2(&pTriangles->m_pVertices[pTriangles->m_pIndices[i+2] * 3]);
    p1 = p1 - og;
    p2 = p2 - og;
    acpVec3 normal = p1 ^ p2;
    normal = normal / normal.length();
    pN[i]   = normal.m_x;
    pN[i+1] = normal.m_y;
    pN[i+2] = normal.m_z;
  }

  return pTriangles;
}


/////////////////////////////////////////////////////////////////////

void acpTag_TRIANGLES::writeData(
  aStreamRef stream
) const
{
  int i;

  // the owner index
  acpShort ownerID(getOwnerID());
  ownerID.writeToStream(stream);

  // the color
  acpByte r((aByte)(m_color.m_x * 254));
  r.writeToStream(stream);
  acpByte g((aByte)(m_color.m_y * 254));
  g.writeToStream(stream);
  acpByte b((aByte)(m_color.m_z * 254));
  b.writeToStream(stream);
  
  // the vertices
  acpInt32 nVertices(m_nVertices);
  nVertices.writeToStream(stream);
  const asReal* pR = m_pVertices;
  for (i = 0; i < m_nVertices * 3; i++, pR++) {
    acpFloat val(*pR);
    val.writeToStream(stream);
  }

  // the indices
  acpInt32 nIndices(m_nIndices);
  nIndices.writeToStream(stream);
  const aInt32* pI = m_pIndices;
  for (i = 0; i < m_nIndices; i++, pI++) {
    acpInt32 val(*pI);
    val.writeToStream(stream);
  }

}


/////////////////////////////////////////////////////////////////////

void acpTag_TRIANGLES::clean()
{
  if (m_pVertices) {
    aMemFree((aMemPtr)m_pVertices);
    m_pVertices = NULL;
  }
  if (m_pNormals) {
    aMemFree((aMemPtr)m_pNormals);
    m_pNormals = NULL;
  }
  if (m_pIndices) {
    aMemFree((aMemPtr)m_pIndices);
    m_pIndices = NULL;
  }
  m_nVertices = 0;
  m_nIndices = 0;
}
