/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEA.c 						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: export definitions for win32 DLL.                  */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* adding opcodes:						   */
/*                                                                 */
/* To add opcodes you need to follow these steps:		   */
/*                                                                 */
/*  1. add the text version of the opcode in aTEA.h		   */
/*  2. add the enum version of the opcode in aTEA.h		   */
/*  3. add the conversion from enum to text (aTEA_TextFromOpcode)  */
/*     in this file.						   */
/*  4. add the conversion from text to enum (aTEA_TextToOpcode)    */
/*     in this file.						   */
/*  5. add the opcode length (aTEA_SetOpCodeLengths) in this file. */
/*  6. Add to aTEA_OpCodeTakesAddress where appropriate.	   */
/*  7. add to the opcode handler to the vm (sTEAvm_Execute) in	   */
/*     file aTEAvmInternal.c.					   */
/*  8. add documentation in the reference on the new opcode.	   */
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

#include "aTEA.h"
#include "aTEAText.h"


#define aTEARAWBYTECOL	24
#define aTEAOPVALCOL	16


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local routines
 */

static void sTEA_Tab(char* line, int pos);
static void sTEA_Hex(char* line, tBYTE op);
static aBool sTEA_GetByte(char* line, 
		   	  char* sub1,
		   	  aStreamRef input);
static aBool sTEA_GetShort(char* line, 
			   char* sub1,
			   char* sub2,
			   aStreamRef input);
static aBool sTEA_GetAddress(char* line, 
			     char* sub1,
			     char* sub2,
			     aStreamRef input);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEA_Tab
 */

void sTEA_Tab(char* line, int pos)
{
  int l, m, i;
  
  l = (int)aStringLen(line);
  
  m = pos - l;

  if (m <= 0)
    return;
    
  for (i = l; i < pos; i++)
    line[i] = ' ';
  
  line[i] = 0;
  
} /* sTEA_Tab */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEA_Hex
 */

void sTEA_Hex(char* line, tBYTE op)
{
  char hexByte[3];
  int num;
  
  num = (op & 0xF0) >> 4;

  if (num > 9)
    hexByte[0] = (char)('A' + num - 10);
  else
    hexByte[0] = (char)('0' + num);

  num = op & 0x0F;

  if (num > 9)
    hexByte[1] = (char)('A' + num - 10);
  else
    hexByte[1] = (char)('0' + num);
  
  hexByte[2] = 0;

  aStringCat(line, hexByte);
  
} /* sTEA_Tab */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEA_GetByte
 */

aBool sTEA_GetByte(char* line, 
		   char* sub1,
		   aStreamRef input)
{
  tBYTE val;
  char num[10];
  
  if (aStream_Read(aStreamLibRef(input), input, (char*)&val, 1, NULL))
    return aFalse;

  sTEA_Tab(sub1, aTEARAWBYTECOL);
  sTEA_Hex(sub1, val);

  aStringFromInt(num, val);
  
  aStringCat(line, num);

  return aTrue;

} /* sTEA_GetByte */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEA_GetShort
 */

aBool sTEA_GetShort(char* line, 
		    char* sub1,
		    char* sub2,
		    aStreamRef input)
{
  tSHORT val;
  char num[10];

  if (aStream_Read(aStreamLibRef(input), input, (char*)&val, 
  		   sizeof(tSHORT), NULL))
    return aFalse;

  /* convert to host byte ordering */
  val = aT2HS(val);

  sTEA_Tab(sub1, aTEARAWBYTECOL);
  sTEA_Hex(sub1, (char)((val & 0xFF00) >> 8));

  sTEA_Tab(sub2, aTEARAWBYTECOL);
  sTEA_Hex(sub2, (char)(val & 0xFF));

  aStringFromInt(num, val);
  aStringCat(line, num);

  return aTrue;

} /* sTEA_GetShort */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTEA_GetAddress
 */

aBool sTEA_GetAddress(char* line, 
		      char* sub1,
		      char* sub2,
		      aStreamRef input)
{
  tADDRESS val;
  char num[10];

  if (aStream_Read(aStreamLibRef(input), input, (char*)&val, 
  		   sizeof(tADDRESS), NULL))
    return aFalse;

  /* convert to host byte ordering */
  val = aT2HA((unsigned short)val);

  sTEA_Tab(sub1, aTEARAWBYTECOL);
  sTEA_Hex(sub1, (char)((val & 0xFF00) >> 8));

  sTEA_Tab(sub2, aTEARAWBYTECOL);
  sTEA_Hex(sub2, (char)(val & 0xFF));

  aStringFromInt(num, val);
  aStringCat(line, num);

  return aTrue;

} /* sTEA_GetAddress */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_RetrieveAddress
 */

tADDRESS aTEA_RetrieveAddress(char* storage)
{
  tADDRESS temp;

  aMemCopy(&temp, storage, sizeof(tADDRESS));
  return aH2TA(temp);

} /* aTEA_RetrieveAddress */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_StoreAddress
 */

void aTEA_StoreAddress(char* storage, 
		       tADDRESS val)
{
  tADDRESS temp = aH2TA(val);
  aMemCopy(storage, &temp, sizeof(tADDRESS));

} /* aTEA_StoreAddress */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_RetrieveStack
 */

tSTACK aTEA_RetrieveStack(char* storage)
{
  tSTACK temp;

  aMemCopy(&temp, storage, sizeof(tSTACK));
  return aH2TA(temp);

} /* aTEA_RetrieveStack */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_StoreStack
 */

void aTEA_StoreStack(char* storage, 
		     tSTACK val)
{
  tSTACK temp = aH2TA(val);
  aMemCopy(storage, &temp, sizeof(tSTACK));

} /* aTEA_StoreStack */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_Disassemble
 */

aErr aTEA_Disassemble(aStreamRef input,
		      aStreamRef error,
		      aStreamRef output,
		      unsigned char* lengths)
{
  /* OPTIMIZE could use lookups, length and be much smaller */

  aErr teaErr = aErrNone;
  int pc;
  int curpc = 0;
  tBYTE op;
  char line[aTEA_DISSEM_MAXLINE];
  char sub1[aTEA_DISSEM_MAXLINE];
  char sub2[aTEA_DISSEM_MAXLINE];
  char num[10];

  /* first, read in the header and dump its information */
  if (teaErr == aErrNone) {
    char header[6];
    aStream_Read(aStreamLibRef(input), input, header, 6, &teaErr);
    if (!((header[0] = 'a')
          && (header[1] = 'T'))) {
      aStream_WriteLine(aStreamLibRef(output), output, "file has bad signature", &teaErr);
      return aErrNone;
    }
    aStringCopy(line, "TEA file, version: ");
    aStringFromInt(num, header[2]);
    aStringCat(line, num);
    aStringCat(line, ".");
    aStringFromInt(num, header[3]);
    aStringCat(line, num);
    aStream_WriteLine(aStreamLibRef(output), output, line, &teaErr);
    aStringCopy(line, "return value size: ");
    aStringFromInt(num, header[4]);
    aStringCat(line, num);
    aStream_WriteLine(aStreamLibRef(output), output, line, &teaErr);
    aStringCopy(line, "parameter size: ");
    aStringFromInt(num, header[5]);
    aStringCat(line, num);
    aStream_WriteLine(aStreamLibRef(output), output, line, &teaErr);
    aStream_WriteLine(aStreamLibRef(output), output, "--------------------------------------", &teaErr);
  }

  while (teaErr == aErrNone) {
    char opName[10];
    unsigned char len;
    aStream_Read(aStreamLibRef(input), input, (char*)&op, 1, &teaErr);
    if (teaErr != aErrNone)
      break;
 
    pc = 1;
    sub1[0] = 0;
    sub2[0] = 0;

    /* write out the current code address at the front of the line with a space */
    aStringCopy(line, "00000");
    aStringFromInt(num, curpc);
    aStringCopy(&line[5 - aStringLen(num)], num);
    aStringCat(line, "   ");

    /* validate the opcode */
    if (op > op_EXIT) {
      if (error != NULL)
        aStream_WriteLine(aStreamLibRef(error), error, 
      			  aTEA_ILLEGAL_OPCODE, NULL);
      teaErr = aErrParse;
    } else {


      /* get the opcode name */
      aTEA_TextFromOpCode(opName, (tOpCode)op);
      aStringCat(line, opName);

      /* find it's length */
      aAssert(lengths);
      len = lengths[(int)op];

      /* handle the data column if there is any */
      if (len > 0) {
        sTEA_Tab(line, aTEAOPVALCOL);
      
        if ((len == 1) && (!sTEA_GetByte(line, sub1, input)))
          teaErr = aErrParse;
        else if (len == 2) {
          if (aTEA_OpCodeTakesAddress((tOpCode)op)) {
            if (!sTEA_GetAddress(line, sub1, sub2, input))
              teaErr = aErrParse;
          } else {
            if (!sTEA_GetShort(line, sub1, sub2, input))
              teaErr = aErrParse;
          }
        }
        pc += len;
      }
    }

    /* now dump the actual memory contents */
    sTEA_Tab(line, aTEARAWBYTECOL);
    sTEA_Hex(line, op);

    if (teaErr == aErrNone)
      aStream_WriteLine(aStreamLibRef(output), output, line, &teaErr);

    if ((teaErr == aErrNone)
        && (aStringLen(sub1) != 0))
      aStream_WriteLine(aStreamLibRef(output), output, sub1, &teaErr);

    if ((teaErr == aErrNone)
        && (aStringLen(sub2) != 0))
      aStream_WriteLine(aStreamLibRef(output), output, sub2, &teaErr);
      
    curpc += pc;

  } /* while */
  
  if (teaErr == aErrEOF)
    teaErr = aErrNone;

  return teaErr;

} /* aTEA_Disassemble */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_IsOpCode
 */

tOpCode aTEA_TextToOpCode(char* text)
{
  /* OPTIMIZE could be made much more efficient */

  if (text != NULL) {
    if (!aStringCompare(text, txt_NOP))		return op_NOP;

    if (!aStringCompare(text, txt_PUSHLB))	return op_PUSHLB;
    if (!aStringCompare(text, txt_PUSHLS))	return op_PUSHLS;
    if (!aStringCompare(text, txt_PUSHMB))	return op_PUSHMB;
    if (!aStringCompare(text, txt_PUSHMBX))	return op_PUSHMBX;
    if (!aStringCompare(text, txt_PUSHMS))	return op_PUSHMS;
    if (!aStringCompare(text, txt_PUSHMSX))	return op_PUSHMSX;
    if (!aStringCompare(text, txt_PUSHSB))	return op_PUSHSB;
    if (!aStringCompare(text, txt_PUSHSBX))	return op_PUSHSBX;
    if (!aStringCompare(text, txt_PUSHSS))	return op_PUSHSS;
    if (!aStringCompare(text, txt_PUSHSSX))	return op_PUSHSSX;
    if (!aStringCompare(text, txt_PUSHSBA))	return op_PUSHSBA;
    if (!aStringCompare(text, txt_PUSHSBAX))	return op_PUSHSBAX;
    if (!aStringCompare(text, txt_PUSHSSA))	return op_PUSHSSA;
    if (!aStringCompare(text, txt_PUSHSSAX))	return op_PUSHSSAX;
    if (!aStringCompare(text, txt_PUSHN))	return op_PUSHN;
    if (!aStringCompare(text, txt_PUSHNX))	return op_PUSHNX;

    if (!aStringCompare(text, txt_CONVBS))	return op_CONVBS;
    if (!aStringCompare(text, txt_CONVSB))	return op_CONVSB;

    if (!aStringCompare(text, txt_POPBM))	return op_POPBM;
    if (!aStringCompare(text, txt_POPBMX))	return op_POPBMX;
    if (!aStringCompare(text, txt_POPSM))	return op_POPSM;
    if (!aStringCompare(text, txt_POPSMX))	return op_POPSMX;
    if (!aStringCompare(text, txt_POPBS))	return op_POPBS;
    if (!aStringCompare(text, txt_POPBSX))	return op_POPBSX;
    if (!aStringCompare(text, txt_POPSS))	return op_POPSS;
    if (!aStringCompare(text, txt_POPSSX))	return op_POPSSX;
    if (!aStringCompare(text, txt_POPBSA))	return op_POPBSA;
    if (!aStringCompare(text, txt_POPBSAX))	return op_POPBSAX;
    if (!aStringCompare(text, txt_POPSSA))	return op_POPSSA;
    if (!aStringCompare(text, txt_POPSSAX))	return op_POPSSAX;
    if (!aStringCompare(text, txt_POPCMD))	return op_POPCMD;
    if (!aStringCompare(text, txt_POPB))	return op_POPB;
    if (!aStringCompare(text, txt_POPS))	return op_POPS;
    if (!aStringCompare(text, txt_POPN))	return op_POPN;
    if (!aStringCompare(text, txt_POPNX))	return op_POPNX;

    if (!aStringCompare(text, txt_DECB))	return op_DECB;
    if (!aStringCompare(text, txt_DECS))	return op_DECS;

    if (!aStringCompare(text, txt_INCB))	return op_INCB;
    if (!aStringCompare(text, txt_INCS))	return op_INCS;

    if (!aStringCompare(text, txt_ADDB))	return op_ADDB;
    if (!aStringCompare(text, txt_ADDS))	return op_ADDS;

    if (!aStringCompare(text, txt_SUBB))	return op_SUBB;
    if (!aStringCompare(text, txt_SUBS))	return op_SUBS;

    if (!aStringCompare(text, txt_NEGB))	return op_NEGB;
    if (!aStringCompare(text, txt_NEGS))	return op_NEGS;

    if (!aStringCompare(text, txt_MULTB))	return op_MULTB;
    if (!aStringCompare(text, txt_MULTS))	return op_MULTS;

    if (!aStringCompare(text, txt_DIVB))	return op_DIVB;
    if (!aStringCompare(text, txt_DIVS))	return op_DIVS;
    if (!aStringCompare(text, txt_MODB))	return op_MODB;
    if (!aStringCompare(text, txt_MODS))	return op_MODS;

    if (!aStringCompare(text, txt_ANDB))	return op_ANDB;
    if (!aStringCompare(text, txt_ANDS))	return op_ANDS;

    if (!aStringCompare(text, txt_ORB))		return op_ORB;
    if (!aStringCompare(text, txt_ORS))		return op_ORS;

    if (!aStringCompare(text, txt_XORB))	return op_XORB;
    if (!aStringCompare(text, txt_XORS))	return op_XORS;

    if (!aStringCompare(text, txt_COMPB))	return op_COMPB;
    if (!aStringCompare(text, txt_COMPS))	return op_COMPS;

    if (!aStringCompare(text, txt_RLB))		return op_RLB;
    if (!aStringCompare(text, txt_RLS))		return op_RLS;
    if (!aStringCompare(text, txt_RRB))		return op_RRB;
    if (!aStringCompare(text, txt_RRS))		return op_RRS;

    if (!aStringCompare(text, txt_BRNEG))	return op_BRNEG;
    if (!aStringCompare(text, txt_BRPOS))	return op_BRPOS;
    if (!aStringCompare(text, txt_BRZ))		return op_BRZ;
    if (!aStringCompare(text, txt_BRNZ))	return op_BRNZ;
    if (!aStringCompare(text, txt_BRC))		return op_BRC;
    if (!aStringCompare(text, txt_BRNC))	return op_BRNC;

    if (!aStringCompare(text, txt_CMPBBR))	return op_CMPBBR;
    if (!aStringCompare(text, txt_CMPSBR))	return op_CMPSBR;

    if (!aStringCompare(text, txt_GOTO))	return op_GOTO;

    if (!aStringCompare(text, txt_CALL))	return op_CALL;
    if (!aStringCompare(text, txt_RETURN))	return op_RETURN;

    if (!aStringCompare(text, txt_FMTBB))	return op_FMTBB;
    if (!aStringCompare(text, txt_FMTBD))	return op_FMTBD;
    if (!aStringCompare(text, txt_FMTBU))	return op_FMTBU;
    if (!aStringCompare(text, txt_FMTBH))	return op_FMTBH;
    if (!aStringCompare(text, txt_FMTSB))	return op_FMTSB;
    if (!aStringCompare(text, txt_FMTSD))	return op_FMTSD;
    if (!aStringCompare(text, txt_FMTSU))	return op_FMTSU;
    if (!aStringCompare(text, txt_FMTSH))	return op_FMTSH;

    if (!aStringCompare(text, txt_EXIT))	return op_EXIT;
  }

  return op_INVALID;

} /* aTEA_IsOpCode */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_TextFromOpCode
 */
void aTEA_TextFromOpCode(char* text, tOpCode opCode)
{
  switch (opCode) {
    case op_NOP:	aStringCopy(text, txt_NOP);	 break;

    case op_PUSHLB:	aStringCopy(text, txt_PUSHLB);	 break;
    case op_PUSHLS:	aStringCopy(text, txt_PUSHLS);	 break;
    case op_PUSHMB:	aStringCopy(text, txt_PUSHMB);	 break;
    case op_PUSHMBX:	aStringCopy(text, txt_PUSHMBX);	 break;
    case op_PUSHMS:	aStringCopy(text, txt_PUSHMS);	 break;
    case op_PUSHMSX:	aStringCopy(text, txt_PUSHMSX);	 break;
    case op_PUSHSB:	aStringCopy(text, txt_PUSHSB);	 break;
    case op_PUSHSBX:	aStringCopy(text, txt_PUSHSBX);	 break;
    case op_PUSHSS:	aStringCopy(text, txt_PUSHSS);	 break;
    case op_PUSHSSX:	aStringCopy(text, txt_PUSHSSX);	 break;
    case op_PUSHSBA:	aStringCopy(text, txt_PUSHSBA);	 break;
    case op_PUSHSBAX:	aStringCopy(text, txt_PUSHSBAX); break;
    case op_PUSHSSA:	aStringCopy(text, txt_PUSHSSA);	 break;
    case op_PUSHSSAX:	aStringCopy(text, txt_PUSHSSAX); break;
    case op_PUSHN:	aStringCopy(text, txt_PUSHN);	 break;
    case op_PUSHNX:	aStringCopy(text, txt_PUSHNX);	 break;

    case op_CONVBS:	aStringCopy(text, txt_CONVBS);	 break;
    case op_CONVSB:	aStringCopy(text, txt_CONVSB);	 break;

    case op_POPBM:	aStringCopy(text, txt_POPBM);	 break;
    case op_POPBMX:	aStringCopy(text, txt_POPBMX);	 break;
    case op_POPSM:	aStringCopy(text, txt_POPSM);	 break;
    case op_POPSMX:	aStringCopy(text, txt_POPSMX);	 break;
    case op_POPBS:	aStringCopy(text, txt_POPBS);	 break;
    case op_POPBSX:	aStringCopy(text, txt_POPBSX);	 break;
    case op_POPSS:	aStringCopy(text, txt_POPSS);	 break;
    case op_POPSSX:	aStringCopy(text, txt_POPSSX);	 break;
    case op_POPBSA:	aStringCopy(text, txt_POPBSA);	 break;
    case op_POPBSAX:	aStringCopy(text, txt_POPBSAX);	 break;
    case op_POPSSA:	aStringCopy(text, txt_POPSSA);	 break;
    case op_POPSSAX:	aStringCopy(text, txt_POPSSAX);	 break;
    case op_POPCMD:	aStringCopy(text, txt_POPCMD);	 break;
    case op_POPB:	aStringCopy(text, txt_POPB);	 break;
    case op_POPS:	aStringCopy(text, txt_POPS);	 break;
    case op_POPN:	aStringCopy(text, txt_POPN);	 break;
    case op_POPNX:	aStringCopy(text, txt_POPNX);	 break;

    case op_DECB:	aStringCopy(text, txt_DECB);	break;
    case op_DECS:	aStringCopy(text, txt_DECS);	break;

    case op_INCB:	aStringCopy(text, txt_INCB);	break;
    case op_INCS:	aStringCopy(text, txt_INCS);	break;

    case op_ADDB:	aStringCopy(text, txt_ADDB);	break;
    case op_ADDS:	aStringCopy(text, txt_ADDS);	break;

    case op_SUBB:	aStringCopy(text, txt_SUBB);	break;
    case op_SUBS:	aStringCopy(text, txt_SUBS);	break;

    case op_NEGB:	aStringCopy(text, txt_NEGB);	break;
    case op_NEGS:	aStringCopy(text, txt_NEGS);	break;

    case op_MULTB:	aStringCopy(text, txt_MULTB);	break;
    case op_MULTS:	aStringCopy(text, txt_MULTS);	break;

    case op_DIVB:	aStringCopy(text, txt_DIVB);	break;
    case op_DIVS:	aStringCopy(text, txt_DIVS);	break;
    case op_MODB:	aStringCopy(text, txt_MODB);	break;
    case op_MODS:	aStringCopy(text, txt_MODS);	break;

    case op_ANDB:	aStringCopy(text, txt_ANDB);	break;
    case op_ANDS:	aStringCopy(text, txt_ANDS);	break;

    case op_ORB:	aStringCopy(text, txt_ORB);	break;
    case op_ORS:	aStringCopy(text, txt_ORS);	break;

    case op_XORB:	aStringCopy(text, txt_XORB);	break;
    case op_XORS:	aStringCopy(text, txt_XORS);	break;

    case op_COMPB:	aStringCopy(text, txt_COMPB);	break;
    case op_COMPS:	aStringCopy(text, txt_COMPS);	break;

    case op_RLB:	aStringCopy(text, txt_RLB);	break;
    case op_RLS:	aStringCopy(text, txt_RLS);	break;
    case op_RRB:	aStringCopy(text, txt_RRB);	break;
    case op_RRS:	aStringCopy(text, txt_RRS);	break;

    case op_BRNEG:	aStringCopy(text, txt_BRNEG);	break;
    case op_BRPOS:	aStringCopy(text, txt_BRPOS);	break;
    case op_BRZ:	aStringCopy(text, txt_BRZ);	break;
    case op_BRNZ:	aStringCopy(text, txt_BRNZ);	break;
    case op_BRC:	aStringCopy(text, txt_BRC);	break;
    case op_BRNC:	aStringCopy(text, txt_BRNC);	break;

    case op_CMPBBR:	aStringCopy(text, txt_CMPBBR);	break;
    case op_CMPSBR:	aStringCopy(text, txt_CMPSBR);	break;

    case op_GOTO:	aStringCopy(text, txt_GOTO);	break;

    case op_CALL:	aStringCopy(text, txt_CALL);	break;
    case op_RETURN:	aStringCopy(text, txt_RETURN);	break;

    case op_FMTBB:	aStringCopy(text, txt_FMTBB);	break;
    case op_FMTBD:	aStringCopy(text, txt_FMTBD);	break;
    case op_FMTBU:	aStringCopy(text, txt_FMTBU);	break;
    case op_FMTBH:	aStringCopy(text, txt_FMTBH);	break;
    case op_FMTSB:	aStringCopy(text, txt_FMTSB);	break;
    case op_FMTSD:	aStringCopy(text, txt_FMTSD);	break;
    case op_FMTSU:	aStringCopy(text, txt_FMTSU);	break;
    case op_FMTSH:	aStringCopy(text, txt_FMTSH);	break;

    case op_EXIT:	aStringCopy(text, txt_EXIT);	break;
    
    default:		aStringCopy(text, "invalid");	break;

  } /* opCode switch */

} /* aTEA_TextFromOpCode */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_OpCodeLength
 */

void aTEA_SetOpCodeLengths(unsigned char* lengths)
{
  lengths[op_NOP] = 0;

  lengths[op_PUSHLB] = 1;
  lengths[op_PUSHLS] = 2;
  lengths[op_PUSHMB] = 2;
  lengths[op_PUSHMBX] = 0;
  lengths[op_PUSHMS] = 2;
  lengths[op_PUSHMSX] = 0;
  lengths[op_PUSHSB] = 1;
  lengths[op_PUSHSBX] = 0;
  lengths[op_PUSHSS] = 1;
  lengths[op_PUSHSSX] = 0;
  lengths[op_PUSHSBA] = 2;
  lengths[op_PUSHSBAX] = 0;
  lengths[op_PUSHSSA] = 2;
  lengths[op_PUSHSSAX] = 0;
  lengths[op_PUSHN] = 1;
  lengths[op_PUSHNX] = 0;

  lengths[op_CONVBS] = 0;
  lengths[op_CONVSB] = 0;
      
  lengths[op_POPBM] = 2;
  lengths[op_POPBMX] = 0;
  lengths[op_POPSM] = 2;
  lengths[op_POPSMX] = 0;
  lengths[op_POPBS] = 1;
  lengths[op_POPBSX] = 0;
  lengths[op_POPSS] = 1;
  lengths[op_POPSSX] = 0;
  lengths[op_POPBSA] = 2;
  lengths[op_POPBSAX] = 0;
  lengths[op_POPSSA] = 2;
  lengths[op_POPSSAX] = 0;
  lengths[op_POPCMD] = 0;
  lengths[op_POPB] = 0;
  lengths[op_POPS] = 0;
  lengths[op_POPN] = 1;
  lengths[op_POPNX] = 0;

  lengths[op_DECB] = 1;
  lengths[op_DECS] = 1;

  lengths[op_INCB] = 1;
  lengths[op_INCS] = 1;
  
  lengths[op_ADDB] = 0;
  lengths[op_ADDS] = 0;

  lengths[op_SUBB] = 0;
  lengths[op_SUBS] = 0;

  lengths[op_NEGB] = 0;
  lengths[op_NEGS] = 0;

  lengths[op_MULTB] = 0;
  lengths[op_MULTS] = 0;
  
  lengths[op_DIVB] = 0;
  lengths[op_DIVS] = 0;
  lengths[op_MODB] = 0;
  lengths[op_MODS] = 0;

  lengths[op_ANDB] = 0;
  lengths[op_ANDS] = 0;
  
  lengths[op_ORB] = 0;
  lengths[op_ORS] = 0;

  lengths[op_XORB] = 0;
  lengths[op_XORS] = 0;
  
  lengths[op_COMPB] = 0;
  lengths[op_COMPS] = 0;

  lengths[op_RLB] = 0;
  lengths[op_RLS] = 0;
  lengths[op_RRB] = 0;
  lengths[op_RRS] = 0;

  lengths[op_BRNEG] = sizeof(tADDRESS);
  lengths[op_BRPOS] = sizeof(tADDRESS);
  lengths[op_BRZ] = sizeof(tADDRESS);
  lengths[op_BRNZ] = sizeof(tADDRESS);
  lengths[op_BRC] = sizeof(tADDRESS);
  lengths[op_BRNC] = sizeof(tADDRESS);
      
  lengths[op_GOTO] = sizeof(tADDRESS);

  lengths[op_CMPBBR] = sizeof(tADDRESS);
  lengths[op_CMPSBR] = sizeof(tADDRESS);
  
  lengths[op_CALL] = sizeof(tADDRESS);
  lengths[op_RETURN] = 0;

  lengths[op_FMTBB] = 0;
  lengths[op_FMTBD] = 0;
  lengths[op_FMTBU] = 0;
  lengths[op_FMTBH] = 0;
  lengths[op_FMTSB] = 0;
  lengths[op_FMTSD] = 0;
  lengths[op_FMTSU] = 0;
  lengths[op_FMTSH] = 0;
  
  lengths[op_EXIT] = 0; /* op_EXIT (must be last) */  

} /* aTEA_SetOpCodeLengths */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_OpCodeTakesAddress
 */

aBool aTEA_OpCodeTakesAddress(tOpCode op)
{
  if ((op == op_GOTO)
      || (op == op_BRNEG)
      || (op == op_BRPOS)
      || (op == op_BRZ)
      || (op == op_BRNZ)
      || (op == op_BRC)
      || (op == op_BRNC)
      || (op == op_CMPBBR)
      || (op == op_CMPSBR)
      || (op == op_CALL))
    return aTrue;

  return aFalse;

} /* aTEA_OpCodeTakesAddress */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTEA_ValidateCodeHeader
 */

aVMExit aTEA_ValidateCodeHeader(const char* code,
			    	const unsigned int codeSize,
			    	const unsigned int dataSize)

{
  /* code must be header + single return main */
  if (codeSize < aTEA_HEADERSIZE + 5)
    return aVMExitBadCode;

  /* validate the signature */
  if ((code[0] != 'a') || (code[1] != 'T'))
    return aVMExitBadCode;

  /* validate the version */
  if ((code[2] != aTEA_VERSION_MAJOR)
      || (code[3] != aTEA_VERSION_MINOR))
    return aVMExitBadVersion;

  /* validate the data size */
  if (code[5] != (char)dataSize)
    return aVMExitBadCode;

  return aVMExitNormal;

} /* aTEA_ValidateCodeHeader */
