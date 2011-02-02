/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGDPDF.h                                                //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: PDF Graphics Device class.                         //
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

#ifndef _acpGDPDF_H_
#define _acpGDPDF_H_

#include "aGD.h"
#include "acpList.h"
#include "acpString.h"
#include "acpPDFObject.h"

#define aPDFPAGEWIDTH	612
#define aPDFPAGEHEIGHT	792

class acpGDPDF {
  public:
  				acpGDPDF(
  				  const aIOLib ioRef,
  				  const char* pFileName,
  				  const aFileArea eFileArea);
  				~acpGDPDF();

    unsigned int		width()
    				  { return m_nWidth; }
    unsigned int		height()
    				  { return m_nHeight; }

    void			setGD(
    				  aGD* pGD)
    				  { m_pGD = pGD; }
    aGD*			getGD()
    				  { return m_pGD; }

    void			setSize(
    				  const int nWidth,
    				  const int nHeight);
    void			setColor(
    				  aGDP* pSetColorPrimitive);
    void			rect(
    				  aGDP* pRectPrimitive);
    void			line(
    				  aGDP* pRectPrimitive);
    void			bezier(
    				  aGDP* pBezierPrimitive);
    void			text(
    				  aGDP* pTextPrimitive);
    void			image(
    				  aGDP* pImagePrimitive);

    int				textWidth(
    				  const char* pText,
    				  const unsigned int nSize);

    void			nextPage();

  private:
    void			moveTo(
    				  const float x,
    				  const float y);
    void			lineTo(
    				  const float x,
    				  const float y);
    void			ensureLineColor();
    void			ensureFillColor();
    void			stroke();
    void			closeFillStroke();
    void                        ensureFont();
    aIOLib			m_ioRef;
    aGD*			m_pGD;
    unsigned int		m_nWidth;
    unsigned int  		m_nHeight;

    aBool                       m_bFillColorNotSet;
    aBool                       m_bLineColorNotSet;
    unsigned int		m_color;
    unsigned int		m_fillColor;
    unsigned int		m_lineColor;

    acpString			m_fullname;
    aFileArea			m_filearea;
    acpPDFObject*		m_pPageList;
    acpPDFObject*	        m_pPage;
    acpList<acpPDFObject>	m_objects;
    acpList<acpPDFObject>       m_pages;
    acpPDFObject*		m_pXObject;
    acpPDFObject*		m_pFontResources;
    acpPDFObject*		m_pVariableFont;
    acpPDFObject*		m_pFixedFont;
    acpPDFObject*		m_pResources;
    int				m_nFontsUsed;
    int				m_nImages;
//    aFONTDEF			m_fontDef;

  friend class			acpPDFObject;
};

#endif // _acpGDPDF_H_
