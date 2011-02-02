/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpHTMLDocument.cpp                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotShell client.              //
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

#include "acpHTMLDocument.h"

/////////////////////////////////////////////////////////////////////

acpHTMLDocument::acpHTMLDocument(
  aIOLib ioRef,
  const char* pFilename,
  const aFileArea eArea) :
  m_ioRef(ioRef)
{
  aErr e;
  aStream_CreateFileOutput(m_ioRef, pFilename, eArea, &m_stream, &e);

  *this += "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0 "
  	   "Transitional//EN\">";
  *this += "<HTML>";

}
  			

/////////////////////////////////////////////////////////////////////

acpHTMLDocument::~acpHTMLDocument() 
{
  *this += "</HTML>";

  aErr e;
  aStream_Destroy(m_ioRef, m_stream, &e);
}


/////////////////////////////////////////////////////////////////////

const char* acpHTMLDocument::operator+=(
  const char* pRHS) 
{
  aErr e;
  aStream_Write(m_ioRef, m_stream, pRHS, aStringLen(pRHS), &e);

#ifdef aDEBUG
  aStream_WriteLine(m_ioRef, m_stream, "", &e);
#endif // aDEBUG  

  return "";
}
