/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpTag_TRIANGLES.h				           //
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

#ifndef _acpTag_TRIANGLES_H_
#define _acpTag_TRIANGLES_H_

#include "acpPackageTagID.h"
#include "acpPackageTag.h"

#include "aSymonym.h"
#include "acpVec3.h"



/////////////////////////////////////////////////////////////////////
// TRIANGLES = 0x0002

class acpTag_TRIANGLES :
  public acpPackageTag
{
  public:
  			acpTag_TRIANGLES() :
  			  acpPackageTag(aTAG_TRIANGLES, (void*)1),
  			  m_nVertices(0),
  			  m_pVertices(NULL),
  			  m_pNormals(NULL),
  			  m_nIndices(0),
  			  m_pIndices(NULL)
  			  {}
  			~acpTag_TRIANGLES();

    void		setData(
    	                  const aShort nOwnerID,
    			  const acpVec3& color,
    			  const aInt32 nVertices,
    			  const asReal* pVertices,
    			  const aInt32 nIndices,
    			  const aInt32* pIndices);

    acpPackageTag*	factoryFromStream(
    			  aStreamRef stream);

  protected:
    void		writeData(
    			  aStreamRef stream) const; 

  private:
    void		clean();

    acpVec3		m_color;
    aInt32		m_nVertices;
    const asReal*	m_pVertices;
    const asReal*	m_pNormals;
    aInt32		m_nIndices;
    const aInt32*	m_pIndices;

  friend class		acpTriangles;
};

#endif // _acpTag_TRIANGLES_H_
