/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpXmlTag_VRML2.cpp 		 		           //
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

#include "acpXmlTag_VRML2.h"

#include "acpSymPkgImport.h"
#include "acpVRML2.h"
#include "acpTag_TRIANGLES.h"


/////////////////////////////////////////////////////////////////////

bool VRML2::equalGeometry(
  const acpPkgXML* pTag
) const
{
  if (!GEOMETRY::equalGeometry(pTag))
    return false;

  aAssert(pTag);
  VRML2* pOther = (VRML2*)pTag;
  const char* pN1 = m_vrmlFileName;
  const char* pN2 = pOther->m_vrmlFileName;
  return (!aStringCompare(pN1, pN2));
}


/////////////////////////////////////////////////////////////////////

void VRML2::addGeometry()
{
  if (m_vrmlFileName.length()) {
    acpString msg("importing VRML 2 file \"");
    msg += m_vrmlFileName;
    msg += "\"";
    m_pImporter->log(msg);
    try {
      acpTransform t;
      acpVRML2 vrml(m_ioRef, 
    		    m_vrmlFileName,
    		    aFileAreaUser,
    		    m_pImporter->getLogStream(),
    		    m_pImporter->getErrorStream(),
    		    t);
//    		    ((acpSymPkgImport*)m_pImporter)->currentTransform());
    
      // walk the list of triangle sets created by the VRML import
      // and stick them all onto the package as static geometry
      acpTriangleSoup* pTriangleSet;
      acpListIterator<acpTriangleSoup> soup(vrml.triangleSoup());
      while ((pTriangleSet = soup.next())) {
        acpTag_TRIANGLES* pTriangleTag = new acpTag_TRIANGLES();
        aInt32 nVertices;
        asReal* pVertices;
        aInt32 nIndices;
        aInt32* pIndices;
        pTriangleSet->getAllocatedData(&nVertices, &pVertices, 
      				       &nIndices, &pIndices);
        pTriangleTag->setData(m_pImporter->m_nCurrentOwner,
      			      pTriangleSet->getColor(),
      			      nVertices, pVertices,
      			      nIndices, pIndices);

	addPackageTag(pTriangleTag);
      }
    } catch (const acpException& exception) {
      aTokenInfo ti;
      ti.nLine = 0;
      ti.nColumn = 0;
      const char* data[2];
      data[0] = "unknown";
      data[1] = exception.msg();
      m_pImporter->xmlErr(tagFileError, 
  	    	          ti.nLine, 
  	    	          ti.nColumn, 
  	    	          2, 
 	 	          data, 
	  	          m_pImporter);
    }
  }
}


/////////////////////////////////////////////////////////////////////

void VRML2::addToken(
  const aToken* pToken
)
{
  if (pToken->eType == tkString) {
    m_vrmlFileName += pToken->v.string;
  } else {
    tokenError(pToken, "VMRL2 expects a string filename");
  }
}




/////////////////////////////////////////////////////////////////////

bool VRML2::getDescription(
  acpString& description) 
{
  description = "This imports a VRML file as \"triangle soup\".";
  description += "The .wrl file must be in the aUser directory.";
  description += "VRML 2.0 files are currently supported.";
  description += "Often, this format can be exported directly from "
  		 "CAD packages such as Solidworks and Pro-E.";
  description += "Several optimizations are performed to reduce the "
                 "imported data size.";

  return true; 
}


/////////////////////////////////////////////////////////////////////

bool VRML2::getDirectFormat(
  acpString& format,
  acpString& example,
  acpString& exampleNotes) 
{
  format = "String";
  example = "example.wrl";
  exampleNotes = "Imports the geometries in the file \"example.wrl\""
                 "in the aUser directory.";
 
  return true; 
}

