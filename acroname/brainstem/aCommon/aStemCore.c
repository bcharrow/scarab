/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aStemCore.c     	                                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of core access routines for the 	   */
/*		BrainStem modules.	    			   */
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

#include "aCmd.tea"

#include "aUtil.h"
#include "aStemCore.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aStemCore_CmdFilter
 */

aBool aStemCore_CmdFilter(
  const unsigned char module,
  const unsigned char dataLength,
  const char* data,
  void* ref)
{
  int temp = (int)(long)ref;
  unsigned char cmd = (unsigned char)temp;

  if ((dataLength > 0) && ((unsigned char)data[0] == cmd))
    return aTrue;

  return aFalse;

} /* aStemCore_CmdFilter */
