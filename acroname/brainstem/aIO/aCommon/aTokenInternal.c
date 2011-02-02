/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aTokenInternal.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: tokenizer with preprocessor.                       */
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

#include "aOSDefs.h"
#include "aTokenInternal.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static char aToUpper(char* c);

static aBool pushChar(aTokenizer* t, 
		      char c);
static aBool nextChar(aTokenizer* t, 
		      char* pChar);
static aBool skipWhiteSpace(aTokenizer* t, 
			    char* pChar);
static aBool getToken(aTokenizer* t, aTokenInternal* pTokenInternal);
static aBool resolveToken(aTokenizer* t, 
			  aTokenInternal* pTokenInternal,
			  aTokenList* pTokenList);
static aBool handleMacroCall(aTokenizer* t, 
		             aTokenList** ppParams);
static aBool handleMacroDef(aTokenizer* t, 
		            aTokenList** ppParams);
static aBool handleDefine(aTokenizer* t);
static aBool handleInclude(aTokenizer* t);
static aBool handleIfDef(aTokenizer* t, 
			 aBool bNot);
static aBool handleEndIf(aTokenizer* t);
static aBool handlePPDirective(aTokenizer* t, 
			       aTokenInternal* pTokenInternal);

static aErr tokenListSymDeleteProc(void* pData,
			           void* deleteRef);
			           
static aBool sTokenizer_NextLine(aTokenizer* pTokenizer);
static aErr sTokenizer_ParseNextLine(aTokenizer* pTokenizer);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aToUpper
 */

char aToUpper(char* c)
{
  char rv = *c;
  int cv = rv;

  if ((cv >= 'a') && (cv <= 'z'))
    rv = (char)('A' + (cv - 'a'));
  
  *c = rv;

  return rv;

} /* end of aToUpper */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * pushChar
 */

aBool pushChar(aTokenizer* t, char c)
{
  if ((t->nScanCur > 2)
      && (t->pScanBuf[t->nScanCur - 2] == '\\')) {
    t->nScanCur -= 2;
  } else
    t->nScanCur--;

  return aTrue;

} /* pushChar */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * nextChar
 *
 * returns true if there are any more characters available
 * in the current scan line.  Also transplants escape characters.
 */

aBool nextChar(aTokenizer* t, char* pChar)
{
  /* if we are at the end of the line, return false */
  if (t->pScanBuf[t->nScanCur] == 0) {
    return aFalse;
  } else {
    char c, d;

    c = t->pScanBuf[t->nScanCur++];
    switch (c) {
    
    /* handle escape sequences */
    case '\\':

      /* look ahead at the next character */
      if (t->pScanBuf[t->nScanCur] != 0) {
        d = t->pScanBuf[t->nScanCur];

  	switch(d) {

  	case 'n':
  	  c = '\n';
  	  t->nScanCur++;
  	  break;

  	case 't':
  	  c = '\t';
  	  t->nScanCur++;
  	  break;

  	default:
  	  break;

  	} /* switch */

      /* if this is and end-of-line escape, just get the next
       * line here and continue */
      } else {
        if (sTokenizer_NextLine(t)) {
          return nextChar(t, pChar);
        } else
          return aFalse;        
      }

    } /* switch */

    *pChar = c;
    return aTrue;
  }

} /* nextChar */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * skipWhiteSpace
 */
 
 aBool skipWhiteSpace(aTokenizer* t, char* pChar)
 {
   aBool retVal = aTrue;
   aBool bWhite = aTrue;
   char c, d;

   do {

     if (nextChar(t, &c)) {
     
       /* check for non-valid characters */
       if ((unsigned char)c > 127) {
        
         /* report the error for unterminated comment */
         if (t->errProc) {
           const char* msg[2];
           msg[0] = t->pInputFrame->frameName;
           msg[1] = &c;
           t->errProc(tkErrInvalidChar,
                      t->pInputFrame->lineNum,
                      t->pInputFrame->columnNum,
                      2,
                      msg,
                      t->errProcRef);
         }
         continue;
       }

       if (t->ccMap[(int)c] != ccWhiteSpace) {
 
	 /* we may have just got the last character in the line */
 	 retVal = nextChar(t, &d);
 	 if (retVal == aFalse) {
 	   *pChar = c;
 	   return aTrue;
 	 }

         /* check for comments */
         if ((retVal == aTrue) && (c == '/')) {

           /* chew up to the end of the line for // comment */
           if (d == '/') {

	     /* just move to the next line */
	     while(nextChar(t, &c)) ;


           /* chew up the entire comment */
           } else if (d == '*') {
             while (retVal == aTrue) {
               /* the comment is ended */
               if ((c == '*') && (d == '/')) {
                 break;
               }
               c = d;
               retVal = nextChar(t, &d);
               /* comments can be multi-line */
               if (retVal == aFalse) {
                 d = '\n';
                 retVal = sTokenizer_NextLine(t);
               }
             } /* while */

             if ((t->err == aErrEOF)
                 && !((c == '*') && (d == '/'))) {
               /* report the error for unterminated comment */
               if (t->errProc) {
                 const char* msg[1];
                 msg[0] = t->pInputFrame->frameName;
                 t->errProc(tkErrUntermCmnt,
                 	    t->pInputFrame->lineNum,
                 	    t->pInputFrame->columnNum,
                 	    1,
                 	    msg,
                 	    t->errProcRef);
               }
             }
           } else
             goto got_one;

         } else {
got_one:
           if (retVal == aTrue) {
             pushChar(t, d);
           } else {
             if (t->err == aErrEOF)
               retVal = aTrue;
           }
           bWhite = aFalse;
           *pChar = c;
         }
       } 
     } else {
       retVal = aFalse;
     }
   } while ((retVal == aTrue) && (bWhite == aTrue));

   return retVal;

} /* skipWhiteSpace */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * getToken
 */

aBool getToken(
  aTokenizer* t, 
  aTokenInternal* pTokenInternal
)
{
  char c, v;
  char* p;
  int lblLen = 0;
  int strLen = 0;
  aBool bRead = aFalse;
  float divisor = 0;

  /* move past any white space */
  if (!skipWhiteSpace(t, &c))
    return(aFalse);

  /* initialize the token */
  aBZero(pTokenInternal, sizeof(aTokenInternal));

  /* store the start position of the token which is 
   * is one less than the position in the input frame 
   * because we already got the first character
   */
  pTokenInternal->line = t->pInputFrame->lineNum;
  pTokenInternal->column = (unsigned int)t->nScanCur;
  if (pTokenInternal->column == 0)
    pTokenInternal->column++;
  pTokenInternal->pParseFrame = t->pInputFrame;

  /* initialize the parse error */
  t->parseError = prsErrNone;

  /* switch on the first character's code */
  switch (t->ccMap[(int)c]) {
  
    case ccLineEnd:
      pTokenInternal->t.eType = tkNewLine;
      return(aTrue);

    case ccSpecial:
      pTokenInternal->t.eType = tkSpecial;

      /* check for character */
      if (c == '\'') {
      
        /* get the next character */
        if (!nextChar(t, &v)) return aFalse;
        
        /* check for another ' */
        if (!nextChar(t, &c)) return aFalse;
        
        if (c == '\'') {
          pTokenInternal->t.eType = tkInt;
          pTokenInternal->t.v.integer = v;
          return aTrue;
        } else {
          /* restore the state */
	  if (!pushChar(t, c))
	    return aFalse;
	  c = v;
        }

      } /* if (c = '\'') */

      /* check for quoted string */
      else if (c == '"') {
        aErr tkErr = aErrNone;
        aStreamRef buf = NULL;

        /* we start by sticking the string value into the identifier's
         * storage and if the string gets larger than 
         * aINPLACESTRINGMAX we switch to a StreamBuffer
         * which has a length bounded only by memory */
        pTokenInternal->t.v.s.string = pTokenInternal->t.v.s.storage;
        p = pTokenInternal->t.v.s.string;
        if (!nextChar(t, &c)) return aFalse;
	do {
	  
	  switch (c) {
	    
	  /* handle escape sequences within the string */
	  case '\\':
	    /* look ahead */
	    if (nextChar(t, &v)) {

	      switch (v) {

	      case '"':
	      case '\n':
	      case '\t':
		c = v;
		break;

	      default:
		if (!pushChar(t, v))
		  return aFalse;
		break;
	      } /* switch v */
	    } /* if nextChar */
	    break;

	  case '"':
	    /* handle the string's null terminator */
	    if (buf) {
	      c = 0;
	      aStream_Write(t->ioRef, buf, &c, 1, &tkErr);
	      if (tkErr == aErrNone)
	        aStreamBuffer_Get(t->ioRef, buf, NULL,
	        		  &pTokenInternal->t.v.s.string, &tkErr);
	      if (tkErr != aErrNone)
	        return aFalse;
	    } else {
	      *p = 0;
	    }
	    pTokenInternal->t.eType = tkString;
	    return aTrue;
	    break;

	  } // switch

	  /* here we build a stream buffer to handle the stream because
	   * we don't have enough storage in the token structure */
	  if (!buf && (strLen >= (int)aINPLACESTRINGMAX)) {
	    aStreamBuffer_Create(t->ioRef, aMAXIDENTIFIERLEN, &buf, &tkErr);
	    if (tkErr == aErrNone)
	      aStream_Write(t->ioRef, buf, pTokenInternal->t.v.s.string,
	      		    (unsigned long)strLen, &tkErr);
	    if (tkErr == aErrNone)
	      pTokenInternal->t.v.s.stringBuffer = buf;
	  }

	  /* store the new character */
	  if (buf)
	    aStream_Write(t->ioRef, buf, &c, 1, &tkErr);
	  else
	    *p++ = c;
	  
	  /* count the new character */
	  strLen++;

          if (!nextChar(t, &c)) return aFalse;
	} while (t->parseError == prsErrNone);

      } /* if (c == '\"') */

      else if (c == '#') {

        /* handle #define macros */
        if (!getToken(t, pTokenInternal)) return aFalse;
        if (pTokenInternal->t.eType != tkIdentifier) {
          t->parseError = prsErrInvalidPreProc;
          return(aFalse);
        }
        pTokenInternal->t.eType = tkPreProc;
        return(aTrue);

      } else {
      
        if (c == '.') {

          /* look ahead */
          if (!nextChar(t, &c)) 
            return aFalse;

          /* restore the look ahead */
	  if (!pushChar(t, c))
	    return aFalse;

	  /* if it is a digit, we have a float */
          if (t->ccMap[(int)c] == ccDigit)
            goto parseFloat;

          /* restore the character before the look ahead */
          c = '.';
        }

        /* else it is a special character so just save it */
        pTokenInternal->t.v.special = c;
        return(aTrue);
      }
      break;

    case ccDigit:
      pTokenInternal->t.eType = tkInt;
      if (c == '0') {

        /* since c was zero, we can skip it for all number types
         * unless it was the last character in the stream in which
         * case, we need to build a zero 
         */
        if (!nextChar(t, &c))
          return aTrue;

        /* is it a binary number? */
        if (c == 'b') {
          bRead = nextChar(t, &c);
	  while ((bRead == aTrue) && 
	         ((c == '0') || (c == '1'))) {
	    pTokenInternal->t.v.integer <<= 1;
	    if (c == '1')
	      pTokenInternal->t.v.integer += 1;
            bRead = nextChar(t, &c);
	  } /* while */

	  /* eof is a valid token terminator */
	  if ((bRead == aFalse) && (t->err = aErrEOF))
	    return aTrue;

          /* next character should be saved */
          return(pushChar(t, c));

        } /* if (c == 'b') */

        /* is it a hex number ? */
        else if (c == 'x') {

          if (!nextChar(t, &c)) return aFalse;
	  aToUpper(&c);
	  if (!(t->ccMap[(int)c] == ccDigit) &&
	       ((c < 'A') || (c > 'F'))) {
	    t->parseError = prsErrUnknownSym;
	    return aFalse;
	  }
	  do {
	    pTokenInternal->t.v.integer <<= 4;
	    pTokenInternal->t.v.integer += (c <= '9') ? (c - '0') : 
					      (c - 'A' + 10);
            bRead = nextChar(t, &c);
            if (bRead == aTrue)
	      aToUpper(&c);
	  }  while (((t->ccMap[(int)c] == ccDigit) ||
		    ((c >= 'A') && (c <= 'F'))) && 
		     (bRead == aTrue));

	  /* eof is a valid token terminator */
	  if ((bRead == aFalse) && (t->err = aErrEOF))
	    return aTrue;

          /* next character should be saved */
          return(pushChar(t, c));

        } /* else if (c == 'x') */

        /* is it a float? */
        else if (c == '.') {
          goto parseFloat;
        }

        if (t->ccMap[(int)c] == ccWhiteSpace)
          return aTrue;

        /* it is some other character */
        return(pushChar(t, c));

      } /* if (c == '0') */

      /* here we have a number, start by assuming an int */
      do {
        if (c == '.') {
parseFloat:
          /* check to see if we are already in a float */
          if (divisor != 0)
            break;
          pTokenInternal->t.eType = tkFloat;
          pTokenInternal->t.v.floatVal = (float)pTokenInternal->t.v.integer;
          divisor = 0.1f;
        } else if (pTokenInternal->t.eType == tkInt) {
	  pTokenInternal->t.v.integer *= 10;
	  pTokenInternal->t.v.integer += c - '0';
	} else {
	  pTokenInternal->t.v.floatVal += divisor * (c - '0');
	  divisor /= 10;
	}

        bRead = nextChar(t, &c);

      } while (((t->ccMap[(int)c] == ccDigit) || (c == '.')) 
               && (bRead == aTrue));

      /* eof is a valid token terminator */
      if ((bRead == aFalse) && (t->err = aErrEOF))
        return aTrue;

      /* next character should be saved */
      return(pushChar(t, c));

    case ccLetter:
      p = pTokenInternal->t.v.identifier;
      do {
	*p++ = c;
	if (lblLen++ >= aMAXIDENTIFIERLEN) {
	  t->parseError = prsErr;
	  return aFalse;
	}
        bRead = nextChar(t, &c);
      } while (((t->ccMap[(int)c] == ccLetter) || 
                (t->ccMap[(int)c] == ccDigit) ||
                (c == '_')) &&
               (bRead == aTrue));

      if (bRead == aTrue)
        /* put the character back */
        pushChar(t, c);
      else
        /* special case at end of file or line */
        bRead = aTrue;
      

      /* here, the potential identifier or label is done */
      if (bRead == aTrue) {

	/* NULL terminate */
	*p = 0; lblLen++;

	/* set identifier */
	pTokenInternal->t.eType = tkIdentifier;
	return aTrue;

      } /* bRead == aTrue */
      break;

    default:
      return(aFalse);

  } /* t->ccMap switch */

  pTokenInternal->pParseFrame = t->pInputFrame;

  return(aTrue);

} /* getToken */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * resolveToken
 *
 * Performs macro expansion on the token if needed.
 */

aBool resolveToken(aTokenizer* t, 
		   aTokenInternal* pTokenInternal,
		   aTokenList* pTokenList)
{
  aErr findErr;
  aTokenList* pDefineReplacement;

  aAssert(t);
  aAssert(pTokenInternal);

  /* only identifiers and macros potentially expand */
  if ((pTokenInternal->t.eType == tkIdentifier)
      || (pTokenInternal->t.eType == tkMacroMask)) {

    /* try to find it in the symbol table */
    findErr = aSymbolTableInternal_Find(t->pSymbolTable,
    				        pTokenInternal->t.v.identifier,
    				        (void**)&pDefineReplacement);
    if (findErr == aErrNone) {

      /* found it, throw away the define and replace it with the
       * token list.  Also cache the character position and set
       * the entire define expansion to this position */
      unsigned int lineNum = pTokenInternal->line;
      unsigned int columnNum = pTokenInternal->column;

      aErr tkErr = aErrNone;

      /* if we have a macro, accumulate the macro arguments as
       * token lists */
      aTokenList* pArgs = NULL;
      if (pDefineReplacement->pHead
          && (pDefineReplacement->pHead->t.eType & tkMacroMask)) {
        handleMacroCall(t, &pArgs);
      }

      if (pDefineReplacement) {
        aTokenInternal* pReplacement = pDefineReplacement->pHead;

        /* free up the macro being substituted */
        aMemPoolInternal_Free(t->pTokenPool, pTokenInternal);

        /* for each element in the expansion, see if it is a reference
         * and if so, splice in the argument replacement, else try
         * to resolve the token and then insert it */
        do {
          if (pReplacement) {
            aTokenInternal replacement = *pReplacement;
          
            /* clean up the macro bit if necessary */
            replacement.t.eType &= (tkType)~tkMacroMask;
          
            /* references get spliced based on the reference number
             * which corresponds to the index in of the argument in
             * the argument list */
            if (replacement.t.eType == tkReference) {
              aTokenList* pArg = pArgs;
              int i = -1;
              
              /* find the referenced argument */
              while (pArg && (++i != replacement.t.v.integer)) {
                pArg = pArg->pNext;
              }
              
              /* report an error if we don't find the argument */
              if (!pArg || (replacement.t.v.integer != i)) {
                const char* msg[1];
                msg[0] = t->pInputFrame->frameName;
                t->errProc(tkErrBadArgumentList,
                           lineNum,
                           columnNum,
                           1, msg, t->errProcRef);
                tkErr = aErrParse;

	      /* otherwise, just add in the argument's list */
              } else {
                tkErr = aTokenList_AddListCopy(pTokenList,
              				       pArg,
              				       lineNum,
              				       columnNum);
              }
            } else {
              aTokenInternal* pCopy;

              /* create an unlinked token */
              if (tkErr == aErrNone)
                tkErr = aMemPoolInternal_Alloc(t->pTokenList->pTokenPool, 
              				       (void**)&pCopy);
 
              /* copy it from the substitution token (pTemp) */
              if (tkErr == aErrNone) {
                aMemCopy(pCopy, &replacement, sizeof(aTokenInternal));
                pCopy->pNext = NULL; /* break any links */
                pCopy->line = lineNum;
                pCopy->column = columnNum;
              }

	      /* add the resolved token */ 
              resolveToken(t, pCopy, pTokenList);
            }

            pReplacement = pReplacement->pNext;

          } else
            tkErr = aErrNotFound;

        } while (tkErr == aErrNone);
      }
      /* clean up any arguments that wer created for macros */
      if (pArgs)
        aTokenList_Destroy(pArgs);

      return aTrue;
    }
  }

  /* if we don't expand it, add it to the list */
  aTokenList_Add(pTokenList, pTokenInternal);

  return aFalse;

} /* resolveToken */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * handleMacroCall
 * 
 * Here, we already know we need a '(' character and we should 
 * just have comma seperated token lists with a closing ')' or it
 * is an error.
 */

aBool handleMacroCall(aTokenizer* t, 
		      aTokenList** ppArgs)
{
  aErr macroErr = aErrNone;
  aTokenList* pArgs = NULL;
  aTokenList* pList = NULL;
  aTokenInternal token;
  aTokenInternal* pCopy;
  int parens = 1;

  /* handle the intitial '(' character */
  if (!getToken(t, &token)
      || (token.t.eType != tkSpecial)
      || (token.t.v.special != '(')) {
    const char* msg[1];
    msg[0] = t->pInputFrame->frameName;
    t->errProc(tkErrBadArgumentList,
               t->pInputFrame->lineNum,
               (unsigned int)t->nScanCur,
               1, msg, t->errProcRef);
    return aFalse;
  }

  /* now look for the rest */
  while (macroErr == aErrNone) {

    if (!getToken(t, &token)) {
      const char* msg[1];
      msg[0] = t->pInputFrame->frameName;
      t->errProc(tkErrBadArgumentList,
                 t->pInputFrame->lineNum,
                 (unsigned int)t->nScanCur,
                 1, msg, t->errProcRef);

      /* clean out any arguments held since the last error */
      if (pArgs)
        aTokenList_Destroy(pArgs);
      if (pList)
        aTokenList_Destroy(pList);

      return aFalse;
    }

    switch (token.t.eType) {

    case tkSpecial:

      /* keep track of opening and closing parens */
      if (token.t.v.special == '(')
        parens++;
      if (token.t.v.special == ')')
        parens--;
     
      if (token.t.v.special == ';') {
        const char* msg[1];
        msg[0] = t->pInputFrame->frameName;
        t->errProc(tkErrMissingParen,
                   t->pInputFrame->lineNum,
                   (unsigned int)t->nScanCur,
                   1, msg, t->errProcRef);
        if (pArgs)
          aTokenList_Destroy(pArgs);
        if (pList)
          aTokenList_Destroy(pList);
        return aFalse;
      }

      /* see if this finishes the current argument.  If so,
       * add it to the list of arguments */
      if ( ((token.t.v.special == ')') && (parens < 1))
           || (token.t.v.special == ',')) {
        if (pArgs) {
          /* insert it at the and of the argument list */
          aTokenList* pTemp = pArgs;
          while (pTemp->pNext)
            pTemp = pTemp->pNext;
          pTemp->pNext = pList;
        } else {
          /* it is the head of the formerly empty argument list */
          pArgs = pList;
        }
        /* reset the current list to empty */
        pList = NULL;
        /* we are done so bail */
        if (token.t.v.special == ')')
          goto done;
        break;
      }
      /* fall through */

    default:
      /* create the list if we don't have one */
      if (pList == NULL) {
        macroErr = aTokenList_Create(t->pTokenPool, &pList);
        if (macroErr != aErrNone)
          return aFalse;
      }
 
      /* add a copy of the token to the current arg list */
        /* create the copy */
      if (macroErr == aErrNone)
        macroErr = aMemPoolInternal_Alloc(t->pTokenPool, 
        				  (void**)&pCopy);
 
      /* copy it from the parsed token */
      if (macroErr == aErrNone) {
        aMemCopy(pCopy, &token, sizeof(aTokenInternal));
        resolveToken(t, pCopy, pList);
      }

      break;
      
    } /* switch */
  } /* while */

done:

  *ppArgs = pArgs;

  return aTrue;

} /* handleMacroCall */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * handleMacroDef
 * 
 * Here, we already know we follow a '(' character so we should 
 * just have comma seperated identifiers with a closing ')' or it
 * is an error.  On error, we just dump to the end of the line to
 * avoid error explosion.
 */

aBool handleMacroDef(aTokenizer* t, 
		     aTokenList** ppParams)
{
  aErr macroErr = aErrNone;
  aTokenInternal ident;
  aBool ret = aTrue;
  aTokenList* pList = NULL;


  /* get the first token */   
  ret = getToken(t, &ident);

  while (ret && (macroErr == aErrNone)) {

    /* if token is a ')' we are done */
    if (ident.t.eType == tkSpecial) { 
      if (ident.t.v.special == ')')
        break;
      else
        goto error;
    }
        
    if (ident.t.eType != tkIdentifier)
      goto error;

    /* create the list if we don't have one */
    if (pList == NULL) {
      macroErr = aTokenList_Create(t->pTokenPool, &pList);
      if (macroErr != aErrNone)
        return aFalse;
    }
    
    /* add the parameter to the list */
    macroErr = aTokenList_AddCopy(pList, &ident, ident.line, ident.column);

    /* get the next token */
    ret = getToken(t, &ident);

    /* if it was the separating comma, get another token */    
    if ((ident.t.eType == tkSpecial)
        && (ident.t.v.special == ','))
      ret = getToken(t, &ident);

  } /* while */

  *ppParams = pList;

  return ret;

error:
  {
    const char* msg[1];
    msg[0] = t->pInputFrame->frameName;
    t->errProc(tkErrBadArgumentList,
               ident.line,
               ident.column,
               1, msg, t->errProcRef);
  }
  
  if (pList)
    aTokenList_Destroy(pList);
  
  return aFalse;

} /* handleMacroDef */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * handleDefine
 */

aBool handleDefine(aTokenizer* t)
{
  aErr defErr = aErrNone;
  aBool ret;
  aTokenInternal ident;
  aTokenInternal token;
  aTokenList* pMacroParams = NULL;
  aTokenList* pList = NULL;
  aTokenList* pFound = NULL;
  char c;

  aAssert(t);

  /* we don't do defines when hiding */
  if (t->nPPHide > 0)
    return aTrue;

  if (!getToken(t, &ident))
    return aFalse;

  /* make sure it was an identifier */
  if (ident.t.eType != tkIdentifier) {
    t->parseError = prsErrIdentExpected;
    return aFalse;
  }

  /* get the next character and check for macro definition */
  ret = nextChar(t, &c);
  if (ret) {
    if (c == '(') {
      ret = handleMacroDef(t, &pMacroParams);
      /* if the macro succeeded, morph the ident into a macro */
      if (ret) {
        ident.t.eType = tkMacroMask;
      } else {
        const char* msg[2];
        msg[0] = t->pInputFrame->frameName;
        msg[1] = ident.t.v.identifier;
        t->errProc(tkErrBadArgumentList,
                   ident.line,
                   ident.column,
                   2, msg, t->errProcRef);
      }
    } else {
      ret = pushChar(t, c);
    }
  }

  /* scan to the end of the line and build a token list for use 
   * in the substitution */
  while ((defErr == aErrNone) && ret) {
    ret = getToken(t, &token);
    if (ret == aFalse)
      break;
    if (pList == NULL) {
      defErr = aTokenList_Create(t->pTokenPool, &pList);
      if (defErr != aErrNone)
        return aFalse;
    }

    /* check to see if the token is one of the parameters and if
     * so, change it's type to a parameter reference */
    if (pMacroParams && (token.t.eType == tkIdentifier)) {
      aTokenInternal* pTemp = pMacroParams->pHead;
      int i = 0;
      aAssert(pMacroParams);
      
      /* scan the list of parameters to make sure the ident 
       * isn't one */
      while (pTemp) {
        aAssert(pTemp->t.eType == tkIdentifier);
        if (aStringCompare(pTemp->t.v.identifier, 
        		   token.t.v.identifier) == 0) {
          /* if it is a reference, morph it into one and set it's 
           * parameter position */
          token.t.eType = tkReference;
          token.t.v.integer = i;
          break;
        }
        pTemp = pTemp->pNext;
        i++;
      }
    }
    defErr = aTokenList_AddCopy(pList, &token, token.line, token.column);
  }

  /* see if the define was already in the symbol table */
  defErr = aSymbolTableInternal_Find(t->pSymbolTable,
  				     ident.t.v.identifier,
  				     (void**)&pFound);
  if (defErr == aErrNotFound) {

    /* morph the token type into a macro if we have a macro 
     * param list */
    if ((ident.t.eType == tkMacroMask) 
        && pList
        && pList->pHead)
      pList->pHead->t.eType |= tkMacroMask;

    /* add the define to the symbol table */
    aSymbolTableInternal_Insert(t->pSymbolTable, 
  		      	        ident.t.v.identifier,
  		      	        pList,
  			        tokenListSymDeleteProc,
  			        NULL);
  } else {
    /* report the duplicate define and toss the list we have built */
    if (t->errProc) {
      const char* msg[2];
      msg[0] = t->pInputFrame->frameName;
      msg[1] = ident.t.v.identifier;
      t->errProc(tkErrDuplicateDefine,
                 ident.line,
                 ident.column,
                 2, msg, t->errProcRef);
    }
    t->err = aTokenList_Destroy(pList);
  }

  /* clean up the macro parameter list if it was created, we only
   * need the references in the token list for this define to 
   * constuct the macro later */
  if (pMacroParams)
    aTokenList_Destroy(pMacroParams);

  return aTrue;

} /* handleDefine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * handleInclude
 */

aBool handleInclude(aTokenizer* t)
{
  aErr incErr = aErrNone;
  aTokenInternal token;
  char* pFilename;
  char name[aMAXIDENTIFIERLEN];
  aFileArea eArea;
  aBool bNotDone;

  aStreamRef fileStream;
  aParseFrame* pFrame;

  aAssert(t);

  /* we don't do includes when hiding */
  if (t->nPPHide > 0)
    return aTrue;

  /* first, get the included file */
  if (!getToken(t, &token))
    return aFalse;

  /* see if it is a system file */
  if ((token.t.eType == tkSpecial)
      && (token.t.v.special == '<')) {

    name[0] = 0;

    /* glue up the file name */
    bNotDone = aTrue;
    while (bNotDone == aTrue) {
      unsigned int len = (unsigned int)aStringLen(name);
      if (!getToken(t, &token))
        return aFalse;

      switch (token.t.eType) {
      case tkSpecial:
        switch (token.t.v.special) {
        case '>':
          bNotDone = aFalse;
          break;
        /* concatenate legal file characters onto the name */
        case '_':
        case '.':
          if (len + 2 >= aMAXIDENTIFIERLEN) {
            t->parseError = prsErrStrLen;
            return aFalse;
          }
          name[len++] = token.t.v.special;
          name[len] = 0;
          break;
        default:
          t->parseError = prsErrStringExpected;
          return aFalse;
        } /* switch */
        break;
      case tkIdentifier:
        if (len + aStringLen(token.t.v.identifier) 
            >= aMAXIDENTIFIERLEN) {
          t->parseError = prsErrStrLen;
          return aFalse;
        }
        aStringCatSafe(name, aMAXIDENTIFIERLEN, token.t.v.identifier);
        break;
      case tkNewLine:
      default:
        t->parseError = prsErrStringExpected;
        return aFalse;
      } /* switch */
    } /* while */

    pFilename = name;
    eArea = aFileAreaSystem;

  } else {
    /* make sure it was a string */
    if (token.t.eType != tkString) {
      t->parseError = prsErrStringExpected;
      return aFalse;
    }
    pFilename = token.t.v.s.string;
    eArea = t->eIncludeArea;
  }

  /* make sure the include filename isn't too long (incl. NULL) */
  if (aStringLen(pFilename) >= aFILE_NAMEMAXCHARS) {
    const char* data[2];
    data[0] = token.pParseFrame->frameName;
    data[1] = pFilename;
    t->errProc(tkErrFilenameLength,
      	       token.line,
      	       token.column,
      	       2, data, t->errProcRef);
    if (token.t.v.s.stringBuffer)
      aStream_Destroy(t->ioRef, token.t.v.s.stringBuffer, NULL);
    return aFalse;
  }

  /* try to find the file being included */
  if (aStream_CreateFileInput(aStreamLibRef(t->pInputFrame->input),
  			      pFilename, eArea,
  			      &fileStream, &incErr)) {
    /* format an error message */
    if (t->errProc) {
      const char* data[2];
      data[0] = token.pParseFrame->frameName;
      data[1] = pFilename;
      t->errProc(tkErrIncludeNotFnd,
      		 token.line,
      		 token.column,
      		 2, data, t->errProcRef);
    }
    t->parseError = prsErrFileNotFound;

    /* MRW */
    if (token.t.v.s.stringBuffer)
      aStream_Destroy(t->ioRef, token.t.v.s.stringBuffer, NULL);
    return aFalse;
  }

  /* try to build a parse frame */
  if (aParseFrame_Create(pFilename,
  			 fileStream,
  			 &pFrame,
  			 &incErr)) {
    t->parseError = prsErrUnknown;

    /* MRW */
    if (token.t.v.s.stringBuffer)
      aStream_Destroy(t->ioRef, token.t.v.s.stringBuffer, NULL);
    return aFalse;
  }

  /* set the current line for this parse frame to null */
  t->nScanCur = 0;
  t->pScanBuf[0] = 0;
  t->pInputFrame->lineNum++;

  /* link the new parse frame into the tokenizer */
  aParseFrame_AddChild(t->pInputFrame, pFrame, &incErr);

  /* set the new current parse frame to the included file's */
  t->pInputFrame = pFrame;

  /* MRW */
  if (token.t.v.s.stringBuffer)
    aStream_Destroy(t->ioRef, token.t.v.s.stringBuffer, NULL);
  
  return aTrue;

} /* handleInclude */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * handleIfDef
 */
aBool handleIfDef(aTokenizer* t, aBool bNot)
{
  aBool ret = aFalse;
  aErr findErr;
  aErr tkErr;
  aTokenInternal token;
  aTokenList* pTokenList;

  aAssert(t);

  /* first, get the definition check identifier */
  if (!getToken(t, &token))
    return aFalse;

  /* now we have a new token, see if it is defined */
  if (token.t.eType == tkIdentifier) {
    findErr = aSymbolTableInternal_Find(t->pSymbolTable,
    				token.t.v.identifier,
    				(void**)&pTokenList);
    /* add it to the pp stack */
    if ((findErr == aErrNone) || (findErr == aErrNotFound)) {
      aPPStack* pPP;
      
      tkErr = aMemPoolInternal_Alloc(t->pPPPool, (void**)&pPP);
      if (tkErr == aErrNone) {
        ret = aTrue;
        aStringCopySafe(pPP->identifier, aMAXIDENTIFIERLEN, token.t.v.identifier);
        pPP->pNext = t->pPPStack;

        /* note that this hides input until the endif */
        if (findErr == aErrNotFound) {
          if (bNot == aFalse) {
            t->nPPHide++;
            pPP->bHides = aTrue;
          } else {
            pPP->bHides = aFalse;
          }
        } else {
          if (bNot == aTrue) {
            t->nPPHide++;
            pPP->bHides = aTrue;
          } else {
            pPP->bHides = aFalse;
          }
        }

        t->pPPStack = pPP;
      }
    }
  }

  return ret;

} /* handleIfDef */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * handleEndIf
 */
aBool handleEndIf(aTokenizer* t)
{
  aBool ret = aFalse;

  /* pop it off the stack */
  if (t->pPPStack) {
    aPPStack* pStack = t->pPPStack;
    t->pPPStack = t->pPPStack->pNext;
    if (pStack->bHides == aTrue)
      t->nPPHide--;
    aMemPoolInternal_Free(t->pPPPool, pStack);
   ret = aTrue;
  }
  
  return ret;

} /* handleEndIf */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * handlePPDirective
 */

aBool handlePPDirective(aTokenizer* t, 
			aTokenInternal* pTokenInternal)
{
  aBool ret = aTrue;

  aAssert(t);
  aAssert(pTokenInternal);
  aAssert(pTokenInternal->t.eType == tkPreProc);

  if (aStringCompare(pTokenInternal->t.v.identifier, "define") == 0) {
    if (!handleDefine(t))
      ret = aFalse;
  } else if (aStringCompare(pTokenInternal->t.v.identifier, "include") == 0) {
    if (!handleInclude(t))
      ret = aFalse;
  } else if (aStringCompare(pTokenInternal->t.v.identifier, "ifdef") == 0) {
    if (!handleIfDef(t, aFalse))
      ret = aFalse;
  } else if (aStringCompare(pTokenInternal->t.v.identifier, "ifndef") == 0) {
    if (!handleIfDef(t, aTrue))
      ret = aFalse;
  } else if (aStringCompare(pTokenInternal->t.v.identifier, "endif") == 0) {
    if (!handleEndIf(t))
      ret = aFalse;
  } else {
    /* generate error */
    t->parseError = prsErrInvalidPreProc;
    ret = aFalse;
  }

  /* need to free the directive now that it is handled */
  aMemPoolInternal_Free(t->pTokenPool, pTokenInternal);

  return ret;

} /* handlePPDirective */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_Create
 */

aErr aTokenList_Create(aMemPool* pTokenPool,
		       aTokenList** ppList)
{
  aErr tkErr = aErrNone;
  aTokenList* pList;

  aAssert(pTokenPool);
  aAssert_aMemPoolSize(pTokenPool, sizeof(aTokenInternal));
  aAssert(ppList);

  if (tkErr == aErrNone) {
    pList = (aTokenList*)aMemAlloc(sizeof(aTokenList));
    if (pList == NULL) {
      tkErr = aErrMemory;
    } else {
      aBZero(pList, sizeof(aTokenList));
      pList->pTokenPool = pTokenPool;
      *ppList = pList;
    }
  }

  return tkErr;

} /* aTokenList_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_Add
 * 
 * inserts the token at the end of the list
 */

aErr aTokenList_Add(aTokenList* pList,
		    aTokenInternal* pTokenInternal)
{
  aErr tkErr = aErrNone;
  
    /* insert it at the end of the list */
  if (tkErr == aErrNone) {
    pTokenInternal->pNext = NULL;
    if (pList->pHead == NULL) {
      pList->pHead = pTokenInternal;
    } else {
      pList->pTail->pNext = pTokenInternal;
    }
    pList->pTail = pTokenInternal;
  }

  return tkErr;

} /* aTokenList_Add */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_AddFirst
 * 
 * inserts the token at the head of the list
 */

aErr aTokenList_AddFirst(aTokenList* pList,
		         aTokenInternal* pTokenInternal)
{
  aErr tkErr = aErrNone;
  
    /* insert it at the front of the list */
  if (tkErr == aErrNone) {
    pTokenInternal->pNext = pList->pHead;    
    if (pList->pHead == NULL)
      pList->pTail = pTokenInternal;
    pList->pHead = pTokenInternal;
  }

  return tkErr;

} /* aTokenList_AddFirst */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_AddCopy
 * 
 * inserts a copy of the token at the end of the list
 */

aErr aTokenList_AddCopy(aTokenList* pList,
		        const aTokenInternal* pTokenInternal,
		        const unsigned int line,
		        const unsigned int column)
{
  aErr tkErr = aErrNone;
  aTokenInternal* pCopy;

  aAssert(pList);

  /* create the copy */
  if (tkErr == aErrNone)
    tkErr = aMemPoolInternal_Alloc(pList->pTokenPool, (void**)&pCopy);
 
  /* copy it from the passed-in token */
  if (tkErr == aErrNone) {
    aMemCopy(pCopy, pTokenInternal, sizeof(aTokenInternal));
    pCopy->line = line;
    pCopy->column = column;
    tkErr = aTokenList_Add(pList, pCopy);
  }

  return tkErr;  

} /* aTokenList_AddCopy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_AddListCopy
 * 
 * inserts the list at the end
 */

aErr aTokenList_AddListCopy(aTokenList* pList,
		            aTokenList* pAddList,
		            unsigned int line,
		            unsigned int column)
{
  aErr tkErr = aErrNone;

  aTokenInternal* pTokenInternal = pAddList->pHead;
  while((tkErr == aErrNone) && (pTokenInternal != NULL)) {
    aTokenInternal token = *pTokenInternal;
    tkErr = aTokenList_AddCopy(pList, &token, line, column);
    pTokenInternal = pTokenInternal->pNext;
  }
  
  return tkErr;

} /* aTokenList_AddListCopy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_GetFirst
 */

aErr aTokenList_GetFirst(aTokenList* pList,
		         aTokenInternal** ppTokenInternal)
{
  aAssert(pList);
  aAssert(ppTokenInternal);

  if (pList->pHead == NULL)
    return aErrNotFound;
  else {
    *ppTokenInternal = pList->pHead;
    pList->pHead = pList->pHead->pNext;
  }
  
  return aErrNone;

} /* aTokenList_GetFirst */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * tokenListSymDeleteProc
 */

aErr tokenListSymDeleteProc(void* pData,
			    void* deleteRef)
{
  aTokenList* pList = (aTokenList*)pData;
  aAssert(pList);
  return aTokenList_Destroy(pList);

} /* tokenListSymDeleteProc static local routine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_Empty
 */

aErr aTokenList_Empty(aTokenList* pList)
{
  aErr tkErr = aErrNone;
  aTokenInternal* pTokenInternal;

  /* remove all the list elements */
  /* MRW check for any attached string buffers */
  while((tkErr == aErrNone) && (pList->pHead != NULL)) {
    pTokenInternal = pList->pHead;
    if (pTokenInternal->t.eType == tkString) {
      aStreamRef q = pTokenInternal->t.v.s.stringBuffer;
      if (q && pList->pRef)
        aStream_Destroy((aIOLib)pList->pRef, q, NULL);
    }    
    pList->pHead = pTokenInternal->pNext;
    tkErr = aMemPoolInternal_Free(pList->pTokenPool, pTokenInternal);
  }
  
  return(tkErr);

} /* aTokenList_Empty */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenList_Destroy
 */

aErr aTokenList_Destroy(aTokenList* pList)
{
  aErr tkErr = aErrNone;
  
  aAssert(pList);
  aAssert(pList->pTokenPool);
  
  /* remove any other lists chained from this one */
  if ((tkErr == aErrNone) && pList->pNext) {
    tkErr = aTokenList_Destroy(pList->pNext);
    pList->pNext = NULL;
  }

  /* remove the elements */
  if (tkErr == aErrNone)
    tkErr = aTokenList_Empty(pList);
  
  /* blow away the list itself */
  if (tkErr == aErrNone)
    aMemFree((aMemPtr)pList);

  return tkErr;

} /* aTokenList_Destroy */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTokenizer_NextLine
 *
 * Loads the next line into the scan buffer.
 */

aBool sTokenizer_NextLine(aTokenizer* pTokenizer)
{
  aErr tkErr = aErrNone;
  aBool bFoundCR = aFalse;

  aAssert(pTokenizer);
  aAssert(pTokenizer->pScanBuf);

  if (pTokenizer->bScanDone) {
    pTokenizer->err = aErrEOF;
    return aFalse;
  }

  /* sanity check */
  aAssert(pTokenizer->pScanBuf[pTokenizer->nScanCur] == 0);

  /* reset to the beginning of the scan buffer */
  pTokenizer->nScanCur = 0;

  pTokenizer->pInputFrame->lineNum++;

  /* continue while we are error free */
  while (tkErr == aErrNone) {
    char c;

    /* get the next character */
    if (pTokenizer->pInputFrame->bBuffered == aTrue) {
      c = pTokenizer->pInputFrame->buffer;
      pTokenizer->pInputFrame->bBuffered = aFalse;
    } else {
      aStream_Read(aStreamLibRef(pTokenizer->pInputFrame->input), 
    		   pTokenizer->pInputFrame->input, 
  		   &c, 1, &tkErr);
    }

    /* if we got a character, add it to the buffer, extending the
     * buffer if needed */	 
    if (tkErr == aErrNone) {
    
      /* extend when full */
      if (pTokenizer->nScanCur >= pTokenizer->nScanLen) {
        char* pNewBuf;
        pNewBuf = aMemAlloc((aMemSize)(pTokenizer->nScanLen 
        		    + aTOKENSCANBLOCKSIZE));
        if (!pNewBuf)
          tkErr = aErrMemory;
        else {
          /* copy the old to the new, bigger buffer */
          unsigned long i;
          for (i = 0; i < pTokenizer->nScanLen; i++)
            pNewBuf[i] = pTokenizer->pScanBuf[i];
 
          /* update the pointers and clean up */
          pTokenizer->nScanLen += aTOKENSCANBLOCKSIZE;
          aMemFree(pTokenizer->pScanBuf);
          pTokenizer->pScanBuf = pNewBuf;
        }
      }

      /* add the character to the buffer, checking for eol */
      if (tkErr == aErrNone) {

        /*  For the record:
 	 *   Mac text lines end with 0x0D ("\r")
 	 *   Unix text lines end with 0x0A ("\n")
 	 *   DOS text lines end with 0x0D 0x0A ("\r\n")
 	 */
 	if (c == '\n')
          goto done;
        else {
          /* if a character follows a CR, we have a mac end of line 
           * so buffer the new character */
          if (bFoundCR == aTrue) {
            aAssert(pTokenizer->pInputFrame->bBuffered == aFalse);
            pTokenizer->pInputFrame->buffer = c;
            pTokenizer->pInputFrame->bBuffered = aTrue;
            goto done;
          }
          
          if (c == '\r')
            bFoundCR = aTrue;
          else
            pTokenizer->pScanBuf[pTokenizer->nScanCur++] = c;
        } /* else */
      }
    }

    /* if this was the last line of the parse frame, see if there
     * are any others */
    else if (tkErr == aErrEOF) {

      if (pTokenizer->pInputFrame->pParent == NULL) {
        pTokenizer->bScanDone = aTrue;
        pTokenizer->err = aErrEOF;
        goto done;
      } else {
        aParseFrame* pTemp = pTokenizer->pInputFrame;
        pTokenizer->pInputFrame = pTemp->pParent;
        if (pTemp->input != NULL) {
          aStream_Destroy(aStreamLibRef(pTemp->input), 
        		  pTemp->input, &tkErr);
          pTemp->input = NULL;
        }
      }
    }
  }

done:
  /* stick in parse frame's storage */

  /* terminate and set the read pointer to the start of line */
  pTokenizer->pScanBuf[pTokenizer->nScanCur] = 0;
  pTokenizer->nScanCur = 0;

  return aTrue;

} /* sTokenizer_NextLine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * sTokenizer_ParseNextLine
 *
 * Loads the next line into the scan buffer.
 */

aErr sTokenizer_ParseNextLine(aTokenizer* pTokenizer)
{
  aErr tkErr = aErrNone;

  if (sTokenizer_NextLine(pTokenizer)) {
    aTokenInternal* pTokenInternal;

    tkErr = aErrNone;
    
    /* just gobble up all the tokens available in the 
     * line and stuff them into the token list */
    while (tkErr == aErrNone) {

      /* rip through and process pre-processor directives */
      do {
        /* allocate storage for the new token */
        pTokenInternal = NULL;
        tkErr = aMemPoolInternal_Alloc(pTokenizer->pTokenPool, 
      			               (void*)&pTokenInternal);

        /* try to get a token */
        if (!getToken(pTokenizer, pTokenInternal))
          tkErr = aErrNotFound;

        /* none available so clean up */
        if ((tkErr != aErrNone) 
            && (pTokenInternal != NULL)) {
          aMemPoolInternal_Free(pTokenizer->pTokenPool, 
          			pTokenInternal);

        /* if preprocessor input, try to resolve it */
        } else if (pTokenInternal->t.eType == tkPreProc) {
          if (!handlePPDirective(pTokenizer, pTokenInternal))
            tkErr = aErrNotFound;
        } else
          break;

      } while (tkErr == aErrNone);

      /* now, just add the token to the list */
      if (tkErr == aErrNone) {
        if (pTokenizer->nPPHide > 0) {
          tkErr = aMemPoolInternal_Free(pTokenizer->pTokenPool, 
    					pTokenInternal);
        } else {
          resolveToken(pTokenizer, pTokenInternal, 
          	       pTokenizer->pTokenList);
        }
      }
    } /* while */

  } else
    tkErr = pTokenizer->err;

  return tkErr;

} /* sTokenizer_ParseNextLine */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenInternal_GetInfo
 */

aErr aTokenInternal_GetInfo(const aToken* pToken,
			    aTokenInfo* pTokenInfo)
{
  aErr tkErr = aErrNone;
  aTokenInternal* pTokenInternal = (aTokenInternal*)pToken;
  
  if ((pToken == NULL)
      || (pTokenInfo == NULL))
    tkErr = aErrParam;
  
  if (tkErr == aErrNone) {
    pTokenInfo->nLine = pTokenInternal->line;
    pTokenInfo->nColumn = pTokenInternal->column;
    pTokenInfo->pSourceName = pTokenInternal->pParseFrame->frameName;
    pTokenInfo->pSourceLine = NULL;
  }

  return tkErr;

} /* aTokenInternal_GetInfo */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizerInternal_Create
 */

aErr aTokenizerInternal_Create(aStreamRef tokenStream,
			       const char* streamName,
		       	       aFileArea eIncludeArea,
			       ccType* ccMap,
			       aTokenErrProc errProc,
			       void* errProcRef,
		       	       aTokenizer** ppTokenizer)
{
  aErr tkErr = aErrNone;
  aTokenizer* pTokenizer = NULL;
  aParseFrame* pParseFrame;

  aAssert(streamName);

  if ((ppTokenizer == NULL)
      || (tokenStream == NULL)
      || (aStringLen(tokenStream) >= aFILE_NAMEMAXCHARS))
    tkErr = aErrParam;

  if (tkErr == aErrNone)
    aParseFrame_Create(streamName, tokenStream, &pParseFrame, &tkErr);

  if (tkErr == aErrNone) {
    pTokenizer = (aTokenizer*)aMemAlloc(sizeof(aTokenizer));
    if (pTokenizer == NULL)
      tkErr = aErrMemory;
    else {
      aBZero(pTokenizer, sizeof(aTokenizer));
      pTokenizer->ioRef = aStreamLibRef(tokenStream);
      pTokenizer->eIncludeArea = eIncludeArea;
      pTokenizer->ccMap = ccMap;
      pTokenizer->pInputFrame = pParseFrame;
      pTokenizer->pParseFrameRoot = pParseFrame;
      pTokenizer->parseError = prsErrNone;
      pTokenizer->errProc = errProc;
      pTokenizer->errProcRef = errProcRef;
      tkErr = aMemPoolInternal_Create(sizeof(aTokenInternal), 16, 
      			              &pTokenizer->pTokenPool);
      if (tkErr == aErrNone)
        tkErr = aTokenList_Create(pTokenizer->pTokenPool,
        		          &pTokenizer->pTokenList);

      if (tkErr == aErrNone)
        tkErr = aSymbolTableInternal_Create(&pTokenizer->pSymbolTable);
      if (tkErr == aErrNone)
        tkErr = aMemPoolInternal_Create(sizeof(aPPStack), 8, 
      			        &pTokenizer->pPPPool);

      /* create the scan buffer for reading in lines */
      if (tkErr == aErrNone) {
        pTokenizer->pScanBuf = aMemAlloc(aTOKENSCANBLOCKSIZE);
        if (!pTokenizer->pScanBuf)
          tkErr = aErrMemory;
        else {
          /* initialize and pre-load the first line */
          pTokenizer->pScanBuf[0] = 0;
          pTokenizer->nScanLen = aTOKENSCANBLOCKSIZE;
          pTokenizer->nScanCur = 0;
        }
      }

      if (tkErr == aErrNone) {
        *ppTokenizer = pTokenizer;
      } else {
        *ppTokenizer = NULL;
        aTokenizerInternal_Destroy(pTokenizer);
      }
    }
  }

  return tkErr;

} /* aTokenizerInternal_Create */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizerInternal_OutputError
 */

void aTokenizerInternal_OutputError(aTokenizer* pTokenizer,
			    char* description,
			    aStreamRef errorStream)
{
  char msg[32];
  char num[8];
  
  aAssert(pTokenizer);
  aAssert(pTokenizer->pInputFrame);

  aStringCopySafe(msg, 32, "error(");
  aStringFromInt(num, pTokenizer->pInputFrame->lineNum);
  aStringCatSafe(msg, 32, num);
  aStringCatSafe(msg, 32, ":");
  aStringFromInt(num, pTokenizer->pInputFrame->columnNum);
  aStringCatSafe(msg, 32, num);
  aStringCatSafe(msg, 32, "):");
  aStream_WriteLine(aStreamLibRef(errorStream), errorStream, 
  		    msg, NULL);
  aStream_WriteLine(aStreamLibRef(errorStream), errorStream,
  		    description, NULL);

} /* aTokenizerInternal_OutputError */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizerInternal_Next
 *
 * gets an element from the front of the list
 */

aErr aTokenizerInternal_Next(aTokenizer* pTokenizer,
		             aToken** ppToken)
{
  aErr tkErr = aErrNone;
  aTokenInternal* pTokenInternal = NULL;

  aAssert(pTokenizer);
  aAssert(ppToken);
  aAssert(pTokenizer->pTokenList);

  /* try to fill the list with some tokens */
  do {

    /* see if any remain on the token list */
    tkErr = aTokenList_GetFirst(pTokenizer->pTokenList, &pTokenInternal);

    if (tkErr == aErrNotFound)
      tkErr = sTokenizer_ParseNextLine(pTokenizer);

  } while (tkErr == aErrNotFound);

  if (tkErr == aErrNone)
    *ppToken = (aToken*)pTokenInternal;

  return tkErr;

} /* aTokenizerInternal_Next */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizerInternal_PushBack
 *
 * puts the token back at the front of the list
 *
 * a b c
 *
 * get a
 * get b
 * push back b
 * push back a
 *
 */

aErr aTokenizerInternal_PushBack(aTokenizer* pTokenizer,
		      	         aToken* pToken)
{
  aErr tkErr = aErrNone;

  aAssert(pTokenizer);
  aAssert(pToken);
  aAssert(pTokenizer->pTokenList);
  
  if (tkErr == aErrNone)
    tkErr = aTokenList_AddFirst(pTokenizer->pTokenList, 
    				(aTokenInternal*)pToken);

  return tkErr;

} /* aTokenizerInternal_PushBack */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizerInternal_Dispose
 */

aErr aTokenizerInternal_Dispose(aTokenizer* pTokenizer,
		                aToken* pToken)
{
  aErr tkErr = aErrNone;
  aTokenInternal* pTokenInternal = (aTokenInternal*)pToken;

  aAssert(pTokenizer);
  aAssert(pToken);

  /* clean up the string buffer if it is present */
  if ((pTokenInternal->t.eType == tkString)
      && (pTokenInternal->t.v.s.stringBuffer)) {
    aStream_Destroy(pTokenizer->ioRef, 
    		    pTokenInternal->t.v.s.stringBuffer,
    		    NULL);
    pTokenInternal->t.v.s.stringBuffer = NULL;
  }

  tkErr = aMemPoolInternal_Free(pTokenizer->pTokenPool, pToken);
  
  return tkErr;

} /* aTokenizerInternal_Dispose */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aTokenizerInternal_Destroy
 */

aErr aTokenizerInternal_Destroy(aTokenizer* pTokenizer)
{
  aErr tkErr = aErrNone;
  aAssert(pTokenizer);

  /* blow away the parse frames */
  if ((tkErr == aErrNone) 
      && (pTokenizer->pParseFrameRoot != NULL)) {
    aParseFrame_Destroy(pTokenizer->pParseFrameRoot, &tkErr);
  }

  /* blow away the token list */
  if (pTokenizer->pTokenList != NULL)
    tkErr = aTokenList_Destroy(pTokenizer->pTokenList);

  /* blow away the symbol table */
  if (pTokenizer->pSymbolTable != NULL)
    tkErr = aSymbolTableInternal_Destroy(pTokenizer->pSymbolTable);

  /* blow away the token pool */
  if (pTokenizer->pTokenPool != NULL)
    tkErr = aMemPoolInternal_Destroy(pTokenizer->pTokenPool);

  /* blow away the pp stack pool */
  if (pTokenizer->pPPPool != NULL)
    aMemPoolInternal_Destroy(pTokenizer->pPPPool);

  /* blow away the token scan buffer */
  if (pTokenizer->pScanBuf != NULL) {
    aMemFree(pTokenizer->pScanBuf);
    pTokenizer->pScanBuf = NULL;
  }

  /* blow away the tokenizer itself */
  if (tkErr == aErrNone)
    aMemFree((aMemPtr)pTokenizer);
  
  return tkErr;

} /* aTokenizerInternal_Destroy */
