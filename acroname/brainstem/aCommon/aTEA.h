/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTEA.h 						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: export definitions for win32 DLL.                  */
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

#ifndef _aTEA_H_
#define _aTEA_H_

#include "aIO.h"

typedef unsigned char aTEAProcessID;

#define aTEA_DISSEM_MAXLINE	40
#define aTEA_MAXOPCODESIZE	3

#define aTEA_2BYTE_SIGNATURE	"aT"
#define aTEA_VERSION_MAJOR	0
#define aTEA_VERSION_MINOR	9

#define aTEA_HEADERSIZE		6
#define aTEA_HEADERRVPOS	4

/* explicitly signed for systems with unsigned char type */
typedef signed char tBYTE;
#define tBYTE_MAX 127
#define tBYTE_MIN -128

typedef signed short tSHORT;
#define tSHORT_MAX 32767
#define tSHORT_MIN -32768

typedef unsigned short tADDRESS;
#define tADDRESS_MAX 65535
#define tADDRESS_MIN 0

typedef unsigned short tSTACK;
#define tSTACK_MAX 65535
#define tSTACK_MIN 0

typedef unsigned char tOFFSET;

/* stateReg flags */

#define fNeg		0x08
#define fZero		0x04
#define fCarry		0x02
#define mStackMask	(fNeg | fZero | fCarry);

/* exit errors */

typedef enum {
  aVMExitNormal,		/*  0 */
  aVMExitKill,			/*  1 */
  aVMExitStackUnderflow,	/*  2 */
  aVMExitStackOverflow,		/*  3 */
  aVMExitStackError,		/*  4 */
  aVMExitAddressRange,		/*  5 */
  aVMExitDivByZero,		/*  6 */
  aVMExitIOErr,			/*  7 */
  aVMExitBadOpcode,		/*  8 */
  aVMExitBadStartup,		/*  9 */
  aVMExitBadVersion,		/* 10 */
  aVMExitBadCode,		/* 11 */
  aVMExitBadRetVal,		/* 12 */
  aVMExitBadCommand		/* 13 */
} aVMExit;

/* msg formats */

typedef enum {
  aVMMsgRaw,
  aVMMsgASCII,
  aVMMsgBinary,
  aVMMsgOctal,
  aVMMsgDecimal,
  aVMMsgUDecimal,
  aVMMsgHex
} aVMMsgFormat;


/* special memory mapped IO locations */

typedef enum {

  /* locations that end up in the display */
  portSleep = 0x0000,
  portMsgRaw = 0x0001,
  portMsgASCII = 0x0010,
  portMsgBinary = 0x0011,
  portMsgOctal = 0x0012,
  portMsgDecimal = 0x0013,
  portMsgUDecimal = 0x0014,
  portMsgHex = 0x0015

} aMemSpecial;

/* changes to this must also be updated in the firmware as well as
 * sOpLengths in aTEAvmInternal.c must be updated. Also, code must
 * be recompiled or assembled and the documentation must be updated
 */

typedef enum {

  op_INVALID = -1,

  op_NOP = 0,

  op_PUSHLB, 	/* 1 */
  op_PUSHLS,	/* 2 */
  op_PUSHMB,	/* 3 */
  op_PUSHMBX,
  op_PUSHMS,
  op_PUSHMSX,
  op_PUSHSB,
  op_PUSHSBX,
  op_PUSHSS,
  op_PUSHSSX,
  op_PUSHSBA,
  op_PUSHSBAX,
  op_PUSHSSA,
  op_PUSHSSAX,
  op_PUSHN,
  op_PUSHNX,

  op_CONVBS,
  op_CONVSB,

  op_POPBM,
  op_POPBMX,
  op_POPSM,
  op_POPSMX,
  op_POPBS,
  op_POPBSX,
  op_POPSS,
  op_POPSSX,
  op_POPBSA,
  op_POPBSAX,
  op_POPSSA,
  op_POPSSAX,
  op_POPCMD,
  op_POPB,
  op_POPS,
  op_POPN,
  op_POPNX,

  op_DECB,
  op_DECS,

  op_INCB,
  op_INCS,

  op_ADDB,
  op_ADDS,

  op_SUBB,
  op_SUBS,

  op_NEGB,
  op_NEGS,

  op_MULTB,
  op_MULTS,

  op_DIVB,
  op_DIVS,
  op_MODB,
  op_MODS,

  op_ANDB,
  op_ANDS,

  op_ORB,
  op_ORS,

  op_XORB,
  op_XORS,

  op_COMPB,
  op_COMPS,

  op_RLB,
  op_RLS,
  op_RRB,
  op_RRS,

  op_BRNEG, /* control flow opcodes */
  op_BRPOS,
  op_BRZ,
  op_BRNZ,
  op_BRC,
  op_BRNC,

  op_CMPBBR, /* compare and branch */
  op_CMPSBR,

  op_GOTO,
  
  op_CALL,
  op_RETURN,
  
  op_FMTBB, /* ascii formatting opcodes */
  op_FMTBD,
  op_FMTBU,
  op_FMTBH,
  op_FMTSB,
  op_FMTSD,
  op_FMTSU,
  op_FMTSH,

  op_EXIT /* must be last */

} tOpCode;

#define txt_NOP		"nop"

#define txt_PUSHLB	"pushlb"
#define txt_PUSHLS	"pushls"
#define txt_PUSHMB	"pushmb"
#define txt_PUSHMBX	"pushmbx"
#define txt_PUSHMS	"pushms"
#define txt_PUSHMSX	"pushmsx"
#define txt_PUSHSB	"pushsb"
#define txt_PUSHSBX	"pushsbx"
#define txt_PUSHSS	"pushss"
#define txt_PUSHSSX	"pushssx"
#define txt_PUSHSBA	"pushsba"
#define txt_PUSHSBAX	"pushsbax"
#define txt_PUSHSSA	"pushssa"
#define txt_PUSHSSAX	"pushssax"
#define txt_PUSHN	"pushn"
#define	txt_PUSHNX	"pushnx"

#define txt_CONVBS	"convbs"
#define txt_CONVSB	"convsb"

#define txt_POPBM	"popbm"
#define txt_POPBMX	"popbmx"
#define txt_POPSM	"popsm"
#define txt_POPSMX	"popsmx"
#define txt_POPBS	"popbs"
#define txt_POPBSX	"popbsx"
#define txt_POPSS	"popss"
#define txt_POPSSX	"popssx"
#define txt_POPBSA	"popbsa"
#define txt_POPBSAX	"popbsax"
#define txt_POPSSA	"popssa"
#define txt_POPSSAX	"popssax"
#define txt_POPCMD	"popcmd"
#define txt_POPB	"popb"
#define txt_POPS	"pops"
#define txt_POPN	"popn"
#define txt_POPNX	"popnx"

#define txt_DECB	"decb"
#define txt_DECS	"decs"

#define txt_INCB	"incb"
#define txt_INCS	"incs"

#define txt_ADDB	"addb"
#define txt_ADDS	"adds"

#define txt_SUBB	"subb"
#define txt_SUBS	"subs"

#define txt_NEGB	"negb"
#define txt_NEGS	"negs"

#define txt_MULTB	"multb"
#define txt_MULTS	"mults"

#define txt_DIVB	"divb"
#define txt_DIVS	"divs"
#define txt_MODB	"modb"
#define txt_MODS	"mods"

#define txt_ANDB	"andb"
#define txt_ANDS	"ands"

#define txt_ORB		"orb"
#define txt_ORS		"ors"

#define txt_XORB	"xorb"
#define txt_XORS	"xors"

#define txt_COMPB	"compb"
#define txt_COMPS	"comps"

#define txt_RLB		"rlb"
#define txt_RLS		"rls"
#define txt_RRB		"rrb"
#define txt_RRS		"rrs"

#define txt_BRNEG	"brneg"
#define txt_BRPOS	"brpos"
#define txt_BRZ		"brz"
#define txt_BRNZ	"brnz"
#define txt_BRC		"brc"
#define txt_BRNC	"brnc"

#define txt_GOTO	"goto"

#define txt_CMPBBR	"cmpbbr"
#define txt_CMPSBR	"cmpsbr"

#define txt_CALL	"call"
#define txt_RETURN	"return"

#define txt_FMTBB	"fmtbb"
#define txt_FMTBD	"fmtbd"
#define txt_FMTBU	"fmtbu"
#define txt_FMTBH	"fmtbh"
#define txt_FMTSB	"fmtsb"
#define txt_FMTSD	"fmtsd"
#define txt_FMTSU	"fmtsu"
#define txt_FMTSH	"fmtsh"

#define txt_EXIT	"exit"

#ifdef __cplusplus
extern "C" {
#endif

tADDRESS aTEA_RetrieveAddress(char* storage);
void aTEA_StoreAddress(char* storage, 
		       tADDRESS val);
tSTACK aTEA_RetrieveStack(char* storage);
void aTEA_StoreStack(char* storage, 
		       tSTACK val);
tOpCode aTEA_TextToOpCode(char* text);
void aTEA_TextFromOpCode(char* text, tOpCode opCode);
void aTEA_SetOpCodeLengths(unsigned char* lengths);
aErr aTEA_Disassemble(aStreamRef input,
		      aStreamRef error,
		      aStreamRef output,
		      unsigned char* lengths);
aBool aTEA_OpCodeTakesAddress(tOpCode op);
aVMExit aTEA_ValidateCodeHeader(const char* code,
			    	const unsigned int codeSize,
			    	const unsigned int dataSize);

#ifdef __cplusplus
}
#endif

#endif /* _aTEA_H_ */
