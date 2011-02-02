/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSteepText.h						   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Contains all text strings used by steep.           */
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

#ifndef _aSteepText_H_
#define _aSteepText_H_

#define aSTEEP_NAME             	"steep"
#define aSTEEP_VERSION			"1.0"
#define aSTEEP_VERSION_LINE		aSTEEP_NAME" version "aSTEEP_VERSION

#define aSTEEP_ERROR			"Error:"

#define aSTEEP_COMPILE_WELCOME1		"TEA Compiler"
#define aSTEEP_COMPILE_WELCOME2		"(Tiny Embedded Applications)"

#define aSTEEP_MSG_MISSING_PARAM	"incorrect number of parameters"
#define aSTEEP_PP_FAILURE		"pre-processing failed"

#define aSTEEP_FILENAME_LEN		"filename too long"

#define aSTEEP_MSG_USAGE		"usage: steep [-adp] [-o outputfile] inputfile"

#define aSTEEP_OUTPUT_ERR		"output error"

#define aSTEEP_SUCCESS			"steep succeeded"
#define aSTEEP_FAILED			"steep resulted in %d errors"

#define aSE_ROUT_OR_DECL_EXPECTED	"routine or declaration expected"
#define aSE_IDENTIFIER_ALREADY_DEFINED	"identifier already defined"
#define aSE_TERMINATOR_EXPECTED		"\";\" expected"
#define aSE_SYMBOL_DEFINED		"symbol already defined: "
#define aSE_BAD_ARG_LIST		"bad argument list for macro "
#define aSE_MISSING_PAREN		"missing paren"
#define aSE_INVALID_CHAR		"invalid character in source file: "

#define aTE_UNKNOWN			"unknown error"
#define aTE_UNRECOGNIZED_INPUT		"unrecognized input"
#define aTE_BYTE_EXPECTED		"char expected"
#define aTE_SHORT_EXPECTED		"short expected"
#define aTE_ADDRESS_EXPECTED		"address expected"
#define aTE_NEWLINE_EXPECTED		"newline expected"
#define aTE_NUMBER_RANGE		"number out of range"
#define aTE_INTERNAL			"internal error"
#define aTE_INTERNAL_MEM		"internal memory error"
#define aTE_UNDEFINED_LABELx		"undefined label, "
#define aTE_UNDEFINED_ROUTINEx		"undefined routine, "
#define aTE_UNDEFINED_SYMBOLx		"undefined symbol, "
#define aTE_EXPRESSION_EXPECTED		"expression expected"
#define aTE_CONST_EXPR_EXPECTED		"constant expression expected"
#define aTE_OPCODE_EXPECTED		"opcode expected"
#define aTE_UNDEFINED_SYMx		"undefined symbol: "
#define aTE_SYMBOL_EXPECTED		"symbol expected"
#define aTE_MAIN_MISSING		"main routine missing"
#define aTE_RETURN_REQUIRED		"return value required"
#define aTE_EXPR_NOT_ASSIGN		"expression not assignable"
#define aTE_INCOMPAT_TYPE		"incompatible type"
#define aTE_INCOMPAT_SIGN		"signed/unsigned missmatch"
#define aTE_CONSTANT_RELATION		"relation evaluation never changes"
#define aTE_ILLEGAL_CONST_OP		"illegal constant operation"
#define aTE_EXPR_DIV_ZERO		"expression divides by zero"
#define aTE_TOTAL_ERRORS		" total errors"
#define aTE_ARGUMENT			"argument "
#define aTE_WRONGTYPE			" is wrong type"
#define aTE_INCOMPAT_RETTYPE		"incompatible return type"
#define aTE_TOO_FEW_ARGS		"not enough arguments"
#define aTE_TOO_MANY_ARGS		"too many arguments"
#define aTE_INVALID_ROUTINE_CALL	"invalid routine call"
#define aTE_EXPRESSION_EXPECTED		"expression expected"
#define aTE_INVALID_SHIFT_AMOUNT	"invalid shift amount"
#define aTE_INVALID_CAST		"invalid cast"
#define aTE_BAD_CASE			"constant must follow case"
#define aTE_TYPE_SPECIFIED		"type already specified"
#define aTE_CASE_OUTSIDE_SWITCH		"case outside switch"
#define aTE_DUPLICATE_CASE		"duplicate case in switch"
#define aTE_BREAK_OUTSIDE_BREAKABLE	"can't break unless in switch or loop"
#define aTE_VARIABLE_NOT_INITED		"variable not initialized"
#define	aTE_BYTE_PARAM_EXPECTED		"byte operand expected"
#define	aTE_SHORT_PARAM_EXPECTED	"short operand expected"
#define	aTE_ADDR_PARAM_EXPECTED		"address operand expected"
#define aTE_INCLUDE_NOT_FOUND		"include file not found: "
#define aTE_MISSING_GOTO_ADDRESS	"missing label identifier for goto"
#define aTE_CONTINUE_NOT_IN_LOOP	"continue is outside of loop"
#define aTE_IDENTIFIER_EXPECTED		"identifier expected, found "

#define aTE_UNTERMINATED_COMMENT	"unterminated comment"

/* reserved words */
#define aTR_ASM				"asm"
#define aTR_MAIN			"main"
#define aTR_RETURN			"return"
#define aTR_CONTINUE			"continue"
#define aTR_BREAK			"break"
#define aTR_GOTO			"goto"
#define aTR_IF				"if"
#define aTR_ELSE			"else"
#define aTR_SWITCH			"switch"
#define aTR_WHILE			"while"
#define aTR_DO				"do"
#define aTR_FOR				"for"
#define aTR_CASE			"case"
#define aTR_DEFAULT			"default"

#endif /* _aSteepText_H_ */
