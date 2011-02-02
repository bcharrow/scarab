/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aLeafText.h						   */
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

#ifndef _aLeafText_H_
#define _aLeafText_H_

#define aLEAF_NAME             		"leaf"
#define aLEAF_VERSION			"1.0"
#define aLEAF_VERSION_LINE		aLEAF_NAME" version "aLEAF_VERSION

#define aLEAF_ERROR			"Error:"

#define aLEAF_COMPILE_WELCOME1		"LEAF Compiler"
#define aLEAF_COMPILE_WELCOME2		"(Little Embedded Application Fragments)"

#define aLEAF_MSG_MISSING_PARAM		"incorrect number of parameters"
#define aLEAF_PP_FAILURE		"pre-processing failed"

#define aLEAF_FILENAME_LEN		"filename too long"

#define aLEAF_MSG_USAGE			"usage: leaf [-adp] [-o outputfile] inputfile"

#define aLEAF_OUTPUT_ERR		"output error"

#define aTE_SYMBOL_DEFINED		"symbol already defined: "
#define aTE_BAD_ARG_LIST		"bad argument list for macro "
#define aTE_INVALID_CHAR		"invalid character in source file: "

#define aLEAF_ILLEGAL_OPCODE		" is illegal modification operation"
#define aLEAF_MODOP_EXPECTED		"modification operation expected"
#define aLEAF_MSGINDEX_USED		"message already defined"
#define aLEAF_VECTORINDEX_USED		"vector already defined"
#define aLEAF_MODULE_OUTOFRANGE		"module address out of range"
#define aLEAF_MODULE_ODD		"module address cannot be odd"
#define aLEAF_MSGINDEX_OUTOFRANGE	"message index out of range"
#define aLEAF_VECTORINDEX_OUTOFRANGE	"vector index out of range"
#define aLEAF_MSGSIZE_OVERRANGE		"message contains too many bytes"
#define aLEAF_MSGSIZE_UNDERRANGE	"message must contain one byte"
#define aLEAF_VECTORSIZE_OVERRANGE	"too many references in vector"
#define aLEAF_MSGINDEX_NOTUSED		"message not defined"
#define aLEAF_MSGOFFSET_RANGE		"message modification out of range"
#define aLEAF_UNDEFINED_IDENT		" not defined"
#define aLEAF_CONSTANT_BYTE_EXPECTED	"constant value expected"
#define aLEAF_ADDRESS_RANGE_ERROR	"address must be char"
#define aTE_UNTERMINATED_COMMENT	"unterminated comment"
#define aTE_INCLUDE_NOT_FOUND		"include file not found: "
#define aLEAF_INVALID_CAST		"invalid cast"
#define aLEAF_MODULE_EXPECTED		"module declaration expected"

/* reserved words */
#define aLR_MODULE			"module"
#define aLR_MESSAGE			"message"
#define aLR_VECTOR			"vector"
#define aLR_PREFIX			"prefix"
#define aLR_SUFFIX			"suffix"

#endif /* _aLeafText_H_ */
