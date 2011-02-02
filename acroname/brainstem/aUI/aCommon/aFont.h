/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aFont.h                                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Copyright 1994-2008. Acroname Inc.                              */
/*                                                                 */
/* This software is the property of Acroname Inc.  Any             */
/* distribution, sale, transmission, or re-use of this code is     */
/* strictly forbidden except with permission from Acroname Inc.    */
/*                                                                 */
/* To the full extent allowed by law, Acroname Inc. also excludes  */
/* for itself and its suppliers any liability, wheither based in   */
/* contract or tort (including negligence), for direct,            */
/* incidental, consequential, indirect, special, or punitive       */
/* damages of any kind, or for loss of revenue or profits, loss of */
/* business, loss of information or data, or other financial loss  */
/* arising out of or in connection with this software, even if     */
/* Acroname Inc. has been advised of the possibility of such       */
/* damages.                                                        */
/*                                                                 */
/* Acroname Inc.                                                   */
/* www.acroname.com                                                */
/* 720-564-0373                                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef _aFont_H_
#define _aFont_H_

typedef struct aFont {

  /* Font size. */
  unsigned char size;

  /* Number of bytes per row of the glyph bitmap. */
  unsigned char rowsize;

  /* Number of rows for each bitmap. */
  unsigned char rows;

  /* Spacing between characters. */
  unsigned char descent;

  /* Glyph size = (rowsize * rows + flag byte). */
  unsigned int nGlyphSize;

  /* Indirection table from ASCII code to glyph.  This allows
   * for mapping of multiple characters to a single glyph such
   * as "A" and "a". 
   */
  unsigned char glyph[128];

  /* Number of actual implemented glyphs. */
  unsigned int nGlyphs;

  unsigned char* pGlyphs;

} aFont;


#endif /* _aFont_H_ */
