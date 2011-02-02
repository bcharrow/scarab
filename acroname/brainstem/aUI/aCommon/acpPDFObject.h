/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPDFObject.h                                            //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: PDF Object tag class.                              //
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

#ifndef _acpPDFObject_H_
#define _acpPDFObject_H_

#include "aIO.h"

#include "acpString.h"
#include "acpList.h"

typedef enum {
  kPDFTag,
  kPDFList,
  kPDFData
} aPDFObjectType;

class acpGDPDF;

class acpPDFObject
{
  public:
  			acpPDFObject(
 			  acpGDPDF* pDoc,
			  acpPDFObject* pParent,
  			  const aPDFObjectType eType = kPDFTag);
  			acpPDFObject(
 			  const char* pType,
  			  aIOLib ioRef);
    virtual		~acpPDFObject();

    void		addDictionary(
    			  const char* text);
    void		addDictionaryLine(
    			  const char* line);

    void		addStream(
    			  const char* text);
    void		addStreamLine(
    			  const char* line);

    void		addDictionaryObject(
    			  acpPDFObject* pSubObject);
    void		write(
    			  aStreamRef output);

    void		setOffset(
    			  const int nOffset)
    			  { m_nOffset = nOffset; }
    int			getOffset()
    			  { return m_nOffset; }

    int			getIndex()
    			  { return m_nIndex; }

  protected:
    aIOLib		m_ioRef;
    int			m_nIndex;
    acpPDFObject*       m_pParent;
    aStreamRef		m_dictionaryBuffer;
    aStreamRef		m_streamBuffer;
    int			m_nOffset;
    aPDFObjectType	m_eType;
    int			m_nLines;
    acpPDFObject*	m_pContent;

    acpString		m_type;    
    bool		m_bSubObject;
    acpList<acpPDFObject> m_subObjects;

  friend class		acpGDPDF;
};

#endif // _acpPDFObject_H_

