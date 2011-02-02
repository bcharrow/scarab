/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpGDPDF.cpp                                              //
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

#include "aVersion.h"
#include "aUtil.h"
#include "acpString.h"
#include "acpCounterStream.h"
#include "aGDOffscreen.h"
#include "acpGDPDF.h"
#include "acpASCIIHexFilter.h"

#define aARIALFIRSTCHAR 30
#define aARIALNUMCHARS 226
#define aARIALASCENT 905
#define aARIALDESCENT -212
#define aARIALCAPHEIGHT 905

static short gARIALWIDTHS[aARIALNUMCHARS] = {
 750,  750,  278,  278,  355,  556,  556,  889,  667,  191,
 333,  333,  389,  584,  278,  333,  278,  278,  556,  556,
 556,  556,  556,  556,  556,  556,  556,  556,  278,  278,
 584,  584,  584,  556,  1015, 667,  667,  722,  722,  667,  
 611,  778,  722,  278,  500,  667,  556,  833,  722,  778,  
 667,  778,  722,  667,  611,  722,  667,  944,  667,  667,  
 611,  278,  278,  278,  469,  556,  333,  556,  556,  500,  
 556,  556,  278,  556,  556,  222,  222,  500,  222,  833,
 556,  556,  556,  556,  333,  500,  278,  556,  500,  722,  
 500,  500,  500,  334,  260,  334,  584,  750,  556,  750,  
 222,  556,  333,  1000, 556,  556,  333,  1000, 667,  333,  
 1000, 750,  611,  750,  750,  222,  222,  333,  333,  350,  
 556,  1000, 333,  1000, 500,  333,  944,  750,  500,  667,  
 278,  333,  556,  556,  556,  556,  260,  556,  333,  737,  
 370,  556,  584,  333,  737,  552,  400,  549,  333,  333,  
 333,  576,  537,  278,  333,  333,  365,  556,  834,  834,
 834,  611,  667,  667,  667,  667,  667,  667,  1000, 722,  
 667,  667,  667,  667,  278,  278,  278,  278,  722,  722,  
 778,  778,  778,  778,  778,  584,  778,  722,  722,  722,  
 722,  667,  667,  611,  556,  556,  556,  556,  556,  556,  
 889,  500,  556,  556,  556,  556,  278,  278,  278,  278,  
 556,  556,  556,  556,  556,  556,  556,  549,  611,  556,  
 556,  556,  556,  500,  556,  500
};


/////////////////////////////////////////////////////////////////////

acpGDPDF::acpGDPDF(
  const aIOLib ioRef,
  const char* pFileName,
  const aFileArea eFileArea
) :
  m_ioRef(ioRef),
  m_bFillColorNotSet(aTrue),
  m_bLineColorNotSet(aTrue),
  m_color(0),
  m_pPage((acpPDFObject*)NULL),
  m_pXObject((acpPDFObject*)NULL),
  m_pFontResources((acpPDFObject*)NULL),
  m_pVariableFont((acpPDFObject*)NULL),
  m_pFixedFont((acpPDFObject*)NULL),
  m_pResources((acpPDFObject*)NULL),
  m_nFontsUsed(0),
  m_nImages(0)
{
  m_nWidth = aPDFPAGEWIDTH;
  m_nHeight = aPDFPAGEHEIGHT;
  m_fullname = pFileName;
  m_filearea = eFileArea;

  // add in the basic info object (1)
  acpPDFObject* pInfoTag = 
  	new acpPDFObject(this, NULL);
  acpString l("/Creator (Acroname aUI Library ");
  l += (int)aVERSION_MAJOR;
  l += '.';
  l += (int)aVERSION_MINOR;
  l += " Build ";
  l += (int)aUI_BUILD_NUM;
  l += ")";
  pInfoTag->addDictionaryLine(l);

  // add in the catalog object (2)
  acpPDFObject* pCatalog = 
  	new acpPDFObject(this, NULL);
  pCatalog->addDictionaryLine("/Type /Catalog");
  pCatalog->addDictionaryLine("/Pages 3 0 R");

  // add in the catalog object (3)
  m_pPageList = new acpPDFObject(this, NULL);
  m_pPageList->addDictionaryLine("/Type /Pages");

  
  m_pResources = new acpPDFObject("/Resources", m_ioRef);

  // XObject (4)
  m_pXObject = new acpPDFObject(this, NULL);

  // resources used by the page
  m_pResources->addDictionaryLine("/ProcSet [/PDF /Text]");

  l = "/XObject ";
  l += m_pXObject->getIndex();
  l += " 0 R";
  m_pResources->addDictionaryLine(l);

  // Font (5)
//  m_pVariableFont = new acpPDFObject(this, NULL);

  m_pPageList->addDictionaryObject(m_pResources);

  // start a new page (5+)
  nextPage();
}


/////////////////////////////////////////////////////////////////////

acpGDPDF::~acpGDPDF()
{
  acpString l;

  // finally, set the media content box
  if (m_pPage) {
    l.format("/MediaBox [0 0 %d %d]", m_nWidth, m_nHeight);
    m_pPage->addDictionaryLine(l);
  }

  // set the number of pages
  l.format("/Count %d", m_pages.length());
  m_pPageList->addDictionaryLine(l);
  m_pPageList->addDictionary("/Kids [ ");
  acpPDFObject* pPage;
  acpListIterator<acpPDFObject> pages(m_pages);
  while ((pPage = pages.next())) {
    l.format("%d 0 R ", pPage->getIndex());
    m_pPageList->addDictionary(l);
  }
  m_pPageList->addDictionaryLine("]");

  aErr err = aErrNone;
  aStreamRef stream;

  if (err == aErrNone)
    aStream_CreateFileOutput(m_ioRef,
  			     (char*)m_fullname,
  			     m_filearea,
  			     &stream,
  			     &err);
  if (err == aErrNone) {
    acpCounterStream counter(m_ioRef, stream);
    aStreamRef countStream = counter;

    // initial header signature
    if (err == aErrNone)
      aStream_WriteLine(m_ioRef,
    		        countStream,
    		        "%PDF-1.0",
    		        &err);

    // dump the objects
    if (err == aErrNone) {
      acpPDFObject* pObject;
      acpListIterator<acpPDFObject> objects(m_objects);
      while ((pObject = objects.next())) {
        pObject->setOffset(counter.putCount());
        pObject->write(countStream);
      }
    }

    // the xref
    int xref_offset = counter.putCount();
    aStream_WriteLine(m_ioRef, stream, "xref", &err);
    aAssert(err == aErrNone);
    l.format("0 %d", m_objects.length() + 1);
    aStream_WriteLine(m_ioRef, stream, l, &err);
    aAssert(err == aErrNone);
    // cr, lf (needs to be a 2-byte line ending regardless of platform
    l.format("%010d 65535 f%c%c", 0, 13, 10);
    aStream_Write(m_ioRef, stream, l, l.length(), &err);
    aAssert(err == aErrNone);
    if (err == aErrNone) {
      acpPDFObject* pObject;
      acpListIterator<acpPDFObject> objects(m_objects);
      while ((pObject = objects.next())) {
        int offset = pObject->getOffset();
        l.format("%010d 00000 n%c%c", offset, 13, 10);
	aStream_Write(m_ioRef, stream, l, 20, &err);
        aAssert(err == aErrNone);
      }
    }

    // the trailer
    aStream_WriteLine(m_ioRef, stream, "trailer", &err);
    aAssert(err == aErrNone);
    aStream_WriteLine(m_ioRef, stream, "<<", &err);
    aAssert(err == aErrNone);
    l.format("/Size %d", m_objects.length() + 1);
    aStream_WriteLine(m_ioRef, stream, l, &err);
    aAssert(err == aErrNone);
    l.format("/Root %d 0 R", 2);
    aStream_WriteLine(m_ioRef, stream, l, &err);
    aAssert(err == aErrNone);
    l.format("/Info %d 0 R", 1);
    aStream_WriteLine(m_ioRef, stream, l, &err);
    aAssert(err == aErrNone);
    aStream_WriteLine(m_ioRef, stream, ">>", &err);
    aAssert(err == aErrNone);

    // the startxref
    aStream_WriteLine(m_ioRef, stream, "startxref", &err);
    aAssert(err == aErrNone);
    l = xref_offset;
    aStream_WriteLine(m_ioRef, stream, l, &err);
    aAssert(err == aErrNone);

    // the final signature
    if (err == aErrNone)
      aStream_WriteLine(m_ioRef,
    		        stream,
    		        "%%EOF",
    		        &err);
  }

  if (err == aErrNone)
    aStream_Destroy(m_ioRef,
    		    stream,
    		    &err);

  while (!m_pages.isEmpty())
    m_pages.removeHead();
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::setSize(
  const int nWidth,
  const int nHeight
)
{
  m_nWidth = nWidth;
  m_nHeight = nHeight;
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::setColor(
  aGDP* pSetColorPrimitive
)
{
  if (pSetColorPrimitive->f.setColor.color != m_color)
    m_color = pSetColorPrimitive->f.setColor.color;
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::rect(
  aGDP* pRectPrimitive
)
{
  aAssert(pRectPrimitive->eType == kGDPRect);
  aRECT* pr = &pRectPrimitive->f.rect.bounds;

  if (pRectPrimitive->f.rect.bFilled)
    ensureFillColor();
  // always ensure the line color (matters whether filled or not)
  ensureLineColor();

  // build the path
  float yadj = (float)(m_nHeight - pr->y - pr->height);
  acpString l;
  l.format("%03d.%03d %03d.%03d %03d.%03d %03d.%03d re", 
  	  (int)pr->x, (int)((pr->x - (int)pr->x)*1000),
  	  (int)yadj, (int)((yadj - (int)yadj)*1000),
  	  (int)pr->width, (int)((pr->width - (int)pr->width)*1000),
  	  (int)pr->height, (int)((pr->height - (int)pr->height)*1000)
  	 );
  m_pPage->m_pContent->addStreamLine(l);

  if (pRectPrimitive->f.rect.bFilled) {
    closeFillStroke();
  } else {
    stroke();
  }
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::line(
  aGDP* pLinePrimitive
)
{
  aAssert(pLinePrimitive->eType == kGDPLine);
  aPT* p = pLinePrimitive->f.line.pPoints;
  unsigned int nPoints = pLinePrimitive->f.line.nPoints - 1;

  ensureLineColor();

  moveTo(p->x, p->y);
  for (unsigned int i = 0; i < nPoints; i++) {
    p++;
    lineTo(p->x, p->y);
  }
  stroke();
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::bezier(
  aGDP* pBezierPrimitive
)
{
  aAssert(pBezierPrimitive->eType == kGDPBezier);
  aPT* p = pBezierPrimitive->f.bezier.points;

  ensureLineColor();
  moveTo(p[0].x, p[0].y);
  float y1adj = (float)(m_nHeight - p[1].y);
  float y2adj = (float)(m_nHeight - p[2].y);
  float y3adj = (float)(m_nHeight - p[3].y);
  acpString l;
  l.format("%03d.%03d %03d.%03d %03d.%03d %03d.%03d %03d.%03d %03d.%03d c", 
  	  p[1].x, (int)((p[1].x - (int)p[1].x)*1000),
  	  (int)y1adj, (int)((y1adj - (int)y1adj)*1000),
  	  p[2].x, (int)((p[2].x - (int)p[2].x)*1000),
  	  (int)y2adj, (int)((y2adj - (int)y2adj)*1000),
  	  p[3].x, (int)((p[3].x - (int)p[3].x)*1000),
  	  (int)y3adj, (int)((y3adj - (int)y3adj)*1000)
  	 );
  m_pPage->m_pContent->addStreamLine(l);
  stroke();
}


#if 0
/////////////////////////////////////////////////////////////////////

void acpGDPDF::setFont(
  aGDP* pSetFontPrimitive
)
{
  aAssert(pSetFontPrimitive->eType == kGDPSetFont);
  // need to establish what type of font we should create
  m_fontDef = pSetFontPrimitive->f.textdef.def;
}
#endif


/////////////////////////////////////////////////////////////////////

void acpGDPDF::text(
  aGDP* pTextPrimitive
)
{
  acpString l;
  aAssert(pTextPrimitive->eType == kGDPText);

  ensureFont();
  ensureFillColor();

  m_pPage->m_pContent->addStreamLine("BT");
  int font = (m_pGD->m_font.flags & aUIFIXEDWIDTHFONT) ? 2 : 1;
  l.format("/F%d %d Tf", font, pTextPrimitive->f.text.size);
  m_pPage->m_pContent->addStreamLine(l);
  int y = (int)(m_nHeight - 
  	  	(pTextPrimitive->f.text.bounds.y + 
  	   	 (pTextPrimitive->f.text.bounds.height
  	   	  + ((double)pTextPrimitive->f.text.size 
  	   	     * ((double)aARIALASCENT / 1000.0))) / 2));
  int x = pTextPrimitive->f.text.bounds.x;

  switch (pTextPrimitive->f.text.flags) {
  case aUIALIGNCENTER:
    x += (pTextPrimitive->f.text.bounds.width 
          - textWidth(pTextPrimitive->f.text.pText, 
      		      pTextPrimitive->f.text.size)) / 2;
    break;

  case aUIALIGNRIGHT:
    x += (pTextPrimitive->f.text.bounds.width 
    	  - textWidth(pTextPrimitive->f.text.pText, 
      		      pTextPrimitive->f.text.size));
    break;
  }

  l.format("%03d.%03d %03d.%03d Td", 
  	  (int)x, (int)((x - (int)x)*1000),
  	  (int)y, (int)((y - (int)y)*1000));
  m_pPage->m_pContent->addStreamLine(l);
  m_pPage->m_pContent->addStream("(");

  // need to escape out the '(' and ')' characters
  // first, count the number in the text
  int count = 0;
  const char* p = pTextPrimitive->f.text.pText;
  while (*p) {
    if ((*p == ')') || (*p == '('))
      count++;
    p++;
  }
  // now, allocate the space for making the escaped text
  p = pTextPrimitive->f.text.pText;
  char* escaped = (char*)aMemAlloc(aStringLen(p) + count + 1);
  if (escaped) {
    // then, escape it all out and display
    char* e = escaped;
    do {
      if ((*p == ')') || (*p == '('))
	*e++ = '\\';
      *e++ = *p;
    } while (*p++);
    m_pPage->m_pContent->addStream(escaped);
    aMemFree(escaped);
  }
  m_pPage->m_pContent->addStreamLine(") Tj");
  m_pPage->m_pContent->addStreamLine("ET");
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::image(
  aGDP* pImagePrimitive
)
{
  aErr uiErr = aErrNone;
  acpPDFObject* pImage;
  aGD* pGD = pImagePrimitive->f.copy.pCopyGD;
  aGDOS* pGDOS = (aGDOS*)pGD->vpData;
  aAssert(pGD);
  aAssert(pGDOS);
  
  pImage = new acpPDFObject(this, NULL);

  // insert the image into the pages resource dictionary
  int nIndex = pImage->getIndex();
  acpString l;
  m_nImages++;
  l.format("/Im%d %d 0 R", m_nImages, nIndex);
  m_pXObject->addDictionaryLine(l);

  // build up the image object
  pImage->addDictionaryLine("/Type /XObject");
  pImage->addDictionaryLine("/Subtype /Image");
  l.format("/Name /Im%d", m_nImages);
  pImage->addDictionaryLine(l);
  l.format("/Width %d", pGD->r.width);
  pImage->addDictionaryLine(l);
  l.format("/Height %d", pGD->r.height);
  pImage->addDictionaryLine(l);
  pImage->addDictionaryLine("/ColorSpace /DeviceRGB");
  pImage->addDictionaryLine("/BitsPerComponent 8");

#if 1 /* turned on for compressed images */

  pImage->addDictionaryLine("/Filter /FlateDecode");

  /* it needs to be compressed */
  aStreamRef zlibStream;
  if (uiErr == aErrNone)
    aStream_CreateZLibFilter(m_ioRef,
    			     pImage->m_streamBuffer,
    			     aFileModeWriteOnly,
    			     &zlibStream,
    			     &uiErr);

  if (uiErr == aErrNone) {
    unsigned int y;
    char* p;
    unsigned int inc;
    p = pGDOS->pPixels;
    inc = pGD->r.width * 3;
    for (y = 0; ((uiErr == aErrNone) 
  	       && (y < (unsigned int)pGD->r.height)); y++, p += inc) {
      // the raster line
      if (uiErr == aErrNone)
        aStream_Write(m_ioRef,
        	      zlibStream, 
        	      p, inc, &uiErr);
    } // for
  }

  // complete and flush the compressed data to the stream buffer  
  if (uiErr == aErrNone)
    aStream_Destroy(m_ioRef, zlibStream, &uiErr);

#else

  pImage->addDictionaryLine("/Filter /ASCIIHexDecode");

  /* it needs to be encoded */
  acpASCIIHexFilter* pFilter = 
    new acpASCIIHexFilter(m_ioRef, pImage->m_streamBuffer, 60);
  if (uiErr == aErrNone) {
    unsigned int y;
    char* p;
    unsigned int inc;
    p = pGDOS->pPixels;
    inc = pGD->r.width * 3;
    for (y = 0; ((uiErr == aErrNone) 
  	       && (y < (unsigned int)pGD->r.height)); y++, p += inc) {
      // the raster line
      if (uiErr == aErrNone)
        aStream_Write(m_ioRef, pFilter->getStream(), p, inc, &uiErr);
    } // for
  }

  if (uiErr == aErrNone)
    aStream_WriteLine(m_ioRef, pImage->m_streamBuffer, 
          	      ">", &uiErr);

  // complete and flush the compressed data to the stream buffer  
  delete pFilter;
#endif
    
  // now, write the image drawing instruction
  m_pPage->m_pContent->addStreamLine("q");

  // translate and scale from the unit image
  l.format("%d 0 0 %d %d %d cm", 
  	      pImagePrimitive->f.copy.dest.width,
  	      pImagePrimitive->f.copy.dest.height,
  	      pImagePrimitive->f.copy.dest.x,
  	      m_nHeight - (pImagePrimitive->f.copy.dest.y 
  	                + pImagePrimitive->f.copy.dest.height));
  m_pPage->m_pContent->addStreamLine(l);

  // then draw
  l.format("/Im%d Do", m_nImages);
  m_pPage->m_pContent->addStreamLine(l);
  m_pPage->m_pContent->addStreamLine("Q");
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::moveTo(
  const float x,
  const float y
)
{
  static char l[100];
  float yadj = m_nHeight - y;
  aSNPRINTF(l, 100, "%03d.%03d %03d.%03d m", 
  	  (int)x, (int)((x - (int)x)*1000),
  	  (int)yadj, (int)((yadj - (int)yadj)*1000));
  m_pPage->m_pContent->addStreamLine(l);
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::lineTo(
  const float x,
  const float y
)
{
  static char l[100];
  float yadj = m_nHeight - y;
  aSNPRINTF(l, 100, "%03d.%03d %03d.%03d l", 
  	  (int)x, (int)((x - (int)x)*1000),
  	  (int)yadj, (int)((yadj - (int)yadj)*1000));
  m_pPage->m_pContent->addStreamLine(l);
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::ensureLineColor()
{
  if (m_bLineColorNotSet || (m_color != m_lineColor)) {
    char l[30];
    m_bLineColorNotSet = aFalse;
    m_lineColor = m_color;
    aSNPRINTF(l, 30, "%.2f %.2f %.2f RG", 
    	    (float)(((m_lineColor & 0xFF0000) >> 16) / 255.0),
    	    (float)(((m_lineColor & 0xFF00) >> 8) / 255.0),
    	    (float)((m_lineColor & 0xFF) / 255.0));
    m_pPage->m_pContent->addStreamLine(l);
  }
}


/////////////////////////////////////////////////////////////////////
    
void acpGDPDF::ensureFillColor()
{
  if (m_bFillColorNotSet || (m_color != m_fillColor)) {
    char l[30];
    m_bFillColorNotSet = aFalse;
    m_fillColor = m_color;
    aSNPRINTF(l, 30, "%.2f %.2f %.2f rg", 
	    (float)(((m_fillColor & 0xFF0000) >> 16) / 255.0),
	    (float)(((m_fillColor & 0xFF00) >> 8) / 255.0),
	    (float)((m_fillColor & 0xFF) / 255.0));
    m_pPage->m_pContent->addStreamLine(l);
  }
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::stroke()
{
  m_pPage->m_pContent->addStreamLine("S");
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::closeFillStroke()
{
  m_pPage->m_pContent->addStreamLine("b*");
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::ensureFont()
{
  acpString l;

  if ((m_pGD->m_font.flags & aUIFIXEDWIDTHFONT)
      && !m_pFixedFont) {

    // ensure a resource font entry we can add to
    if (!m_pFontResources) {
      m_pFontResources = new acpPDFObject("/Font", m_ioRef);
      m_pResources->addDictionaryObject(m_pFontResources);
    }

    // build the actual main level font definition
    m_pFixedFont = new acpPDFObject(this, NULL);
    m_nFontsUsed++;

    l.format("/F2 %d 0 R", m_pFixedFont->getIndex());
    m_pFontResources->addDictionaryLine(l);

    m_pFixedFont->addDictionaryLine("/Type /Font");
    m_pFixedFont->addDictionaryLine("/Subtype /Type1");
    m_pFixedFont->addDictionaryLine("/Name /F2");
    m_pFixedFont->addDictionaryLine("/BaseFont /Courier");
  }

  if (!(m_pGD->m_font.flags & aUIFIXEDWIDTHFONT)
      && !m_pVariableFont) {

    // ensure a resource font entry we can add to
    if (!m_pFontResources) {
      m_pFontResources = new acpPDFObject("/Font", m_ioRef);
      m_pResources->addDictionaryObject(m_pFontResources);
    }

    // build the actual main level font definition
    m_pVariableFont = new acpPDFObject(this, NULL);
    
    // add in the reference to this font in the resources
    m_nFontsUsed++;
    l.format("/F1 %d 0 R", m_pVariableFont->getIndex());
    m_pFontResources->addDictionaryLine(l);
    m_pVariableFont->addDictionaryLine("/Type /Font");
    m_pVariableFont->addDictionaryLine("/Subtype /TrueType");
    m_pVariableFont->addDictionaryLine("/Name /F1");
    m_pVariableFont->addDictionaryLine("/BaseFont /Arial");
    m_pVariableFont->addDictionaryLine("/Encoding /WinAnsiEncoding");
    m_pVariableFont->addDictionaryLine("/FirstChar 30");
    m_pVariableFont->addDictionaryLine("/LastChar 255");
    m_pVariableFont->addDictionaryLine("/Widths [");
    int i = 0;
    while (i < aARIALNUMCHARS) {
      if (i && !(i % 12))
        m_pVariableFont->addDictionaryLine("");
      l.format(" %d", gARIALWIDTHS[i]);
      m_pVariableFont->addDictionary(l);
      i++;
    }
    m_pVariableFont->addDictionaryLine(" ]");
    acpPDFObject* m_pVariableFontDescriptor = new acpPDFObject(this, NULL);
    l.format("/FontDescriptor %d 0 R", m_pVariableFontDescriptor->getIndex());
    m_pVariableFontDescriptor->addDictionaryLine("/Type /FontDescriptor");
    m_pVariableFontDescriptor->addDictionaryLine("/FontName /Arial");
    m_pVariableFontDescriptor->addDictionaryLine("/FontBBox [ -665 -325 2028 1006 ]");
    m_pVariableFontDescriptor->addDictionaryLine("/Ascent 905");
    m_pVariableFontDescriptor->addDictionaryLine("/Descent -212");
    m_pVariableFontDescriptor->addDictionaryLine("/CapHeight 905");
    m_pVariableFontDescriptor->addDictionaryLine("/XHeight 724");
    m_pVariableFontDescriptor->addDictionaryLine("/ItalicAngle 0");
    m_pVariableFontDescriptor->addDictionaryLine("/Flags 32");
    m_pVariableFontDescriptor->addDictionaryLine("/StemV 70");
  }

#if 0  	
  if (!m_bFontSet) {
    acpPDFObject* pFont = new acpPDFObject(this, NULL);
    sprintf(l, "/F1 %d 0 R", pFont->getIndex());
    m_pFont->addDictionaryLine(l);
    m_bFontSet = true;
    pFont->addDictionaryLine("/Type /Font");
    pFont->addDictionaryLine("/Subtype /TrueType");
    pFont->addDictionaryLine("/Name /F1");
    pFont->addDictionaryLine("/BaseFont /Arial");
    pFont->addDictionaryLine("/Encoding /WinAnsiEncoding");
    pFont->addDictionaryLine("/FirstChar 30");
    pFont->addDictionaryLine("/LastChar 255");
    pFont->addDictionaryLine("/Widths [");
    int i = 0;
    while (i < aARIALNUMCHARS) {
      if (i && !(i % 12))
        pFont->addDictionaryLine("");
      sprintf(l, " %d", gARIALWIDTHS[i]);
      pFont->addDictionary(l);
      i++;
    }
    pFont->addDictionaryLine(" ]");
    acpPDFObject* pFontDescriptor = new acpPDFObject(this, NULL);
    sprintf(l, "/FontDescriptor %d 0 R", pFontDescriptor->getIndex());
    pFontDescriptor->addDictionaryLine("/Type /FontDescriptor");
    pFontDescriptor->addDictionaryLine("/FontName /Arial");
    pFontDescriptor->addDictionaryLine("/FontBBox [ -665 -325 2028 1006 ]");
    pFontDescriptor->addDictionaryLine("/Ascent 905");
    pFontDescriptor->addDictionaryLine("/Descent -212");
    pFontDescriptor->addDictionaryLine("/CapHeight 905");
    pFontDescriptor->addDictionaryLine("/XHeight 724");
    pFontDescriptor->addDictionaryLine("/ItalicAngle 0");
    pFontDescriptor->addDictionaryLine("/Flags 32");
    pFontDescriptor->addDictionaryLine("/StemV 70");
  }
#endif
}

/////////////////////////////////////////////////////////////////////

int acpGDPDF::textWidth(
  const char* pText,
  const unsigned int nSize
)
{
  int w = 0;
  // accumulate the device unit sizes of the characters in the string
  if (m_pGD->m_font.flags & aUIFIXEDWIDTHFONT) {
    w = aStringLen(pText) * 600;
  } else {
    const unsigned char* p = (const unsigned char*)pText;
    while(*p) {
      if (*p >= aARIALFIRSTCHAR)
        w += gARIALWIDTHS[*p - aARIALFIRSTCHAR];
      else
        w += 1000;
      p++;
    }
  }
  return (int)((double)w / 1000.0 * (double)nSize);
}


/////////////////////////////////////////////////////////////////////

void acpGDPDF::nextPage()
{
  acpString l;

  if (m_pPage) {
    l.format("/MediaBox [ 0 0 %d %d ]", m_nWidth, m_nHeight);
    m_pPage->addDictionaryLine(l);
  }

  m_pPage = new acpPDFObject(this, m_pPageList);
  m_pages.add(m_pPage);

  // set up what we know to start
  m_pPage->addDictionaryLine("/Type /Page");
  l.format("/Parent %d 0 R", m_pPageList->getIndex());
  m_pPage->addDictionaryLine(l);

#if 0
  // resources used by the page
  m_pPage->addDictionaryLine("/Resources <<");
  m_pPage->addDictionaryLine("/ProcSet [/PDF /Text]");
  aString_SetNumParam(l, "/XObject ^0 0 R", 
    		      m_pXObject->getIndex());
  m_pPage->addDictionaryLine(l);

  // list of indirect font references used by the pages
  aString_SetNumParam(l, "/Font ^0 0 R", m_pFont->getIndex());
  m_pPage->addDictionaryLine(l);
  m_pPage->addDictionaryLine(">>");
#endif

  // each page manages it's own content
  m_pPage->m_pContent = new acpPDFObject(this, m_pPage, kPDFTag);

  // set up the default drawing commands
  m_pPage->m_pContent->addStreamLine("/DeviceRGB CS");
  m_pPage->m_pContent->addStreamLine("0.5 w");

  l.format("/Contents %d 0 R", m_pPage->m_pContent->getIndex());
  m_pPage->addDictionaryLine(l);

  // reset the colors for painting to blow the caches
  m_bFillColorNotSet = aTrue;
  m_bLineColorNotSet = aTrue;
}
