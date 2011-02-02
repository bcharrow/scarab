/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aCCMap.c						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: character map for TEA compiler.                    */
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

#include "aCCMap.h"

#include "aOSDefs.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * initCCMap
 */

aErr initCCMap(ccType* pMap)
{
  if (pMap == NULL)
    return aErrParam;

  /* 0x00 */
  pMap[0] = ccError;		/* NUL */
  pMap[1] = ccError;		/* SOH */
  pMap[2] = ccError;		/* STX */
  pMap[3] = ccError;		/* ETX */
  pMap[4] = ccError;		/* EOT */
  pMap[5] = ccError;		/* ENQ */
  pMap[6] = ccError;		/* ACK */
  pMap[7] = ccError;		/* BEL */
  pMap[8] = ccError;		/* Backspace */
  pMap[9] = ccWhiteSpace;	/* Horiz Tab */
  pMap[10] = ccLineEnd;		/* Line Feed */
  pMap[11] = ccError;		/* Vert Tab */
  pMap[12] = ccError;		/* Form Feed */
  pMap[13] = ccLineEnd;		/* Carriage */
  pMap[14] = ccError;		/* SO */
  pMap[15] = ccError;		/* SI */

  /* 0x10 */
  pMap[16] = ccError;		/* DLE */
  pMap[17] = ccError;		/* DL1 */
  pMap[18] = ccError;		/* DL2 */
  pMap[19] = ccError;		/* DL3 */
  pMap[20] = ccError;		/* DL4 */
  pMap[21] = ccError;		/* NAk */
  pMap[22] = ccError;		/* SYN */
  pMap[23] = ccError;		/* ETB */
  pMap[24] = ccError;		/* CAN */
  pMap[25] = ccError;		/* EM */
  pMap[26] = ccError;		/* SUB */
  pMap[27] = ccError;		/* ESC */
  pMap[28] = ccError;		/* FS */
  pMap[29] = ccError;		/* GS */
  pMap[30] = ccError;		/* RS */
  pMap[31] = ccError;		/* US */

  /* 0x20 */
  pMap[32] = ccWhiteSpace;	/* Space */
  pMap[33] = ccSpecial;		/* ! */
  pMap[34] = ccSpecial;		/* " */
  pMap[35] = ccSpecial;		/* # */
  pMap[36] = ccError;		/* $ */
  pMap[37] = ccSpecial;		/* % */
  pMap[38] = ccSpecial;		/* & */
  pMap[39] = ccSpecial;		/* ' */
  pMap[40] = ccSpecial;		/* ( */
  pMap[41] = ccSpecial;		/* ) */
  pMap[42] = ccSpecial;		/* * */
  pMap[43] = ccSpecial;		/* + */
  pMap[44] = ccSpecial;		/* , */
  pMap[45] = ccSpecial;		/* - */
  pMap[46] = ccSpecial;		/* . */
  pMap[47] = ccSpecial;		/* / */

  /* 0x30 */
  pMap[48] = ccDigit;		/* 0 */
  pMap[49] = ccDigit;		/* 1 */
  pMap[50] = ccDigit;		/* 2 */
  pMap[51] = ccDigit;		/* 3 */
  pMap[52] = ccDigit;		/* 4 */
  pMap[53] = ccDigit;		/* 5 */
  pMap[54] = ccDigit;		/* 6 */
  pMap[55] = ccDigit;		/* 7 */
  pMap[56] = ccDigit;		/* 8 */
  pMap[57] = ccDigit;		/* 9 */
  pMap[58] = ccSpecial;		/* : */
  pMap[59] = ccSpecial;		/* ; */
  pMap[60] = ccSpecial;		/* < */
  pMap[61] = ccSpecial;		/* = */
  pMap[62] = ccSpecial;		/* > */
  pMap[63] = ccError;		/* ? */

  /* 0x40 */
  pMap[64] = ccError;		/* @ */
  pMap[65] = ccLetter;		/* A */
  pMap[66] = ccLetter;		/* B */
  pMap[67] = ccLetter;		/* C */
  pMap[68] = ccLetter;		/* D */
  pMap[69] = ccLetter;		/* E */
  pMap[70] = ccLetter;		/* F */
  pMap[71] = ccLetter;		/* G */
  pMap[72] = ccLetter;		/* H */
  pMap[73] = ccLetter;		/* I */
  pMap[74] = ccLetter;		/* J */
  pMap[75] = ccLetter;		/* K */
  pMap[76] = ccLetter;		/* L */
  pMap[77] = ccLetter;		/* M */
  pMap[78] = ccLetter;		/* N */
  pMap[79] = ccLetter;		/* O */
  
  /* 0x50 */
  pMap[80] = ccLetter;		/* P */
  pMap[81] = ccLetter;		/* Q */
  pMap[82] = ccLetter;		/* R */
  pMap[83] = ccLetter;		/* S */
  pMap[84] = ccLetter;		/* T */
  pMap[85] = ccLetter;		/* U */
  pMap[86] = ccLetter;		/* V */
  pMap[87] = ccLetter;		/* W */
  pMap[88] = ccLetter;		/* X */
  pMap[89] = ccLetter;		/* Y */
  pMap[90] = ccLetter;		/* Z */
  pMap[91] = ccSpecial;		/* [ */
  pMap[92] = ccError;		/* \ */
  pMap[93] = ccSpecial;		/* ] */
  pMap[94] = ccSpecial;		/* ^ */
  pMap[95] = ccLetter;		/* _ */

  /* 0x60 */
  pMap[96] = ccError;		/* _ */
  pMap[97] = ccLetter;		/* a */
  pMap[98] = ccLetter;		/* b */
  pMap[99] = ccLetter;		/* c */
  pMap[100] = ccLetter;		/* d */
  pMap[101] = ccLetter;		/* e */
  pMap[102] = ccLetter;		/* f */
  pMap[103] = ccLetter;		/* g */
  pMap[104] = ccLetter;		/* h */
  pMap[105] = ccLetter;		/* i */
  pMap[106] = ccLetter;		/* j */
  pMap[107] = ccLetter;		/* k */
  pMap[108] = ccLetter;		/* l */
  pMap[109] = ccLetter;		/* m */
  pMap[110] = ccLetter;		/* n */
  pMap[111] = ccLetter;		/* o */

  /* 0x70 */
  pMap[112] = ccLetter;		/* p */
  pMap[113] = ccLetter;		/* q */
  pMap[114] = ccLetter;		/* r */
  pMap[115] = ccLetter;		/* s */
  pMap[116] = ccLetter;		/* t */
  pMap[117] = ccLetter;		/* u */
  pMap[118] = ccLetter;		/* v */
  pMap[119] = ccLetter;		/* w */
  pMap[120] = ccLetter;		/* x */
  pMap[121] = ccLetter;		/* y */
  pMap[122] = ccLetter;		/* z */
  pMap[123] = ccSpecial;	/* { */
  pMap[124] = ccSpecial;	/* | */
  pMap[125] = ccSpecial;	/* } */
  pMap[126] = ccSpecial;	/* ~ */
  pMap[127] = ccError;		/* DEL */

  return aErrNone;

} /* initCCMap */


