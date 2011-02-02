/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotProperty.cpp                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the aRobotShell client Property.     //
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

#include "aUtil.h"
#include "aAssert.h"
#include "acpRobotProperty.h"
#include "acpProperty.h"
#include "acpRobotShell.h"



/////////////////////////////////////////////////////////////////////

acpRobotProperty::~acpRobotProperty() 
{
}


/////////////////////////////////////////////////////////////////////

const char* acpRobotProperty::getDescription()
{
  // go fill the description cache if requested
  if (!m_bDescriptionCached) {
    acpRobotShell* pShell = m_pOwner->m_pShell;
    pShell->getPropertyDescription(&m_description, 
      				   m_pOwner->getID(), m_index);
    m_bDescriptionCached = true;
  }
  return (char*)m_description;
}


/////////////////////////////////////////////////////////////////////

int acpRobotProperty::getNegInt()
{
  int val = 0;
  aErr e = aErrNone;
  aToken* pToken = NULL;

  acpRobotShell* pShell = m_pOwner->m_pShell;

  if (!aTokenizer_Next(pShell->m_ioRef,
		       pShell->m_tokenizer,
		       &pToken,
		       &e)) {
    switch (pToken->eType) {
    case tkInt:
      val = pToken->v.integer;
      break;
    case tkFloat:
      val = (int)pToken->v.floatVal;
      break;
    }
    val = -val;

    // clean up
    aTokenizer_Dispose(pShell->m_ioRef,
		       pShell->m_tokenizer,
		       pToken,
		       &e);
  }
  return val;
}


/////////////////////////////////////////////////////////////////////

float acpRobotProperty::getNegFloat()
{
  float val = 0.0f;
  aErr e = aErrNone;
  aToken* pToken = NULL;

  acpRobotShell* pShell = m_pOwner->m_pShell;

  if (!aTokenizer_Next(pShell->m_ioRef,
		       pShell->m_tokenizer,
		       &pToken,
		       &e)) {
    switch (pToken->eType) {
    case tkInt:
      val = (float)pToken->v.integer;
      break;
    case tkFloat:
      val = pToken->v.floatVal;
      break;
    }
    val = -val;

    // clean up
    aTokenizer_Dispose(pShell->m_ioRef,
		       pShell->m_tokenizer,
		       pToken,
		       &e);
  }
  return val;
}


/////////////////////////////////////////////////////////////////////

void acpRobotProperty::set()
{
  acpRobotShell* pShell = m_pOwner->m_pShell;

  aAssert(pShell->m_bConnected);
 
  if (m_typeFlags & aPROPERTY_FLAG_WRITE) {
    aErr e;
    aToken* pToken;
    if (!aTokenizer_Next(pShell->m_ioRef, pShell->m_tokenizer, 
  		         &pToken, &e)) {

      // handle the type conversions, etc.
      int propertyType = m_typeFlags & aPROPERTY_TYPE_MASK;

      switch(propertyType) {

      case aPROPERTY_FLAG_INT: 
        {
          int val;
          switch (pToken->eType) {
          case tkSpecial:
            // could be a negative value
            if (pToken->v.special == '-') {
              val = getNegInt();
            }
            break;
          case tkInt:
            val = pToken->v.integer;
            break;
          case tkFloat:
            val = (int)pToken->v.floatVal;
            break;
          case tkString:
            aIntFromString(&val, pToken->v.string);
            break;
          } // switch
          pShell->sendChar('W');
          pShell->sendInt(m_pOwner->getID());
          pShell->sendShort((short)m_index);
          pShell->sendInt(aPROPERTY_FLAG_INT);
          pShell->sendInt(val, true);
        }
        break;

      case aPROPERTY_FLAG_FLOAT:
        {
          float val;
          switch (pToken->eType) {
          case tkSpecial:
            // could be a negative value
            if (pToken->v.special == '-') {
              val = getNegFloat();
            }
            break;
          case tkInt:
            val = (float)pToken->v.integer;
            break;
          case tkFloat:
            val = pToken->v.floatVal;
            break;
          case tkString:
            sscanf("%f", pToken->v.string, &val);
            break;
          } // switch
          pShell->sendChar('W');
          pShell->sendInt(m_pOwner->getID());
          pShell->sendShort((short)m_index);
          pShell->sendInt(aPROPERTY_FLAG_FLOAT);
          pShell->sendFloat(val, true);
        }
        break;

      case aPROPERTY_FLAG_STRING:
        {
          acpString val;
          switch (pToken->eType) {
          case tkInt: 
            {
              char s[100];
              aStringFromInt(s, pToken->v.integer);
              val = s;          	
            }
            break;
          case tkFloat:
            {
              char s[100];
              sprintf(s, "%f", pToken->v.floatVal);
              val = s;          	
            }
            break;
          case tkString:
            val = pToken->v.string;
            break;
          } // switch
          pShell->sendChar('W');
          pShell->sendInt(m_pOwner->getID());
          pShell->sendShort((short)m_index);
          pShell->sendInt(aPROPERTY_FLAG_STRING);
          pShell->sendString(val, true);
        }
        break;

      case aPROPERTY_FLAG_BOOL:
        break;

      } // switch

      // clean up
      aTokenizer_Dispose(pShell->m_ioRef, pShell->m_tokenizer, 
      		         pToken, &e);
    } else {
      pShell->badInput("set needs a value");
    }
  } else {
    acpString msg(m_name);
    msg += " is not a writable property";
    pShell->badInput(msg);
  }
}



/////////////////////////////////////////////////////////////////////

void acpRobotProperty::get()
{
  acpRobotShell* pShell = m_pOwner->m_pShell;

  aAssert(pShell->m_bConnected);

  if (m_typeFlags & aPROPERTY_FLAG_READ) {

    if (1) {

      // handle the type conversions, etc.

      acpValue v;
      char buff[20];
      int propertyType = m_typeFlags & aPROPERTY_TYPE_MASK;

      switch(propertyType) {

      case aPROPERTY_FLAG_BOOL: 
        {
          int i;
          pShell->sendChar('R');
          pShell->sendInt(m_pOwner->getID());
          pShell->sendShort((short)m_index);
          pShell->sendInt(aPROPERTY_FLAG_BOOL, true);
          i = (int)pShell->nextByte();
          if (i) {
            v.set("true");
          } else {
            v.set("false");
          }
        }
        break;

      case aPROPERTY_FLAG_INT: 
        {
          int i;
          pShell->sendChar('R');
          pShell->sendInt(m_pOwner->getID());
          pShell->sendShort((short)m_index);
          pShell->sendInt(aPROPERTY_FLAG_INT, true);
          i = pShell->nextInt();
          aStringFromInt(buff, i);
          v.set(buff);
        }
        break;

      case aPROPERTY_FLAG_FLOAT:
        {
          float f;
          pShell->sendChar('R');
          pShell->sendInt(m_pOwner->getID());
          pShell->sendShort((short)m_index);
          pShell->sendInt(aPROPERTY_FLAG_FLOAT, true);
          f = pShell->nextFloat();
          aString_FormatFloat(f, buff);
          v.set(buff);
        }
        break;

      case aPROPERTY_FLAG_STRING:
        {
          acpString s;
          pShell->sendChar('R');
          pShell->sendInt(m_pOwner->getID());
          pShell->sendShort((short)m_index);
          pShell->sendInt(aPROPERTY_FLAG_STRING, true);
          pShell->nextString(&s);
          v.set((char*)s);
        }
        break;

      default:
        break;

      } // switch

      if ((v.getType() == acpValue::kString) && v.getStringVal()) {
        if (aStringLen(v.getStringVal())) {
          printf("%s\n", v.getStringVal());
        } else {
          printf("(empty string)\n");
        }
      } else {
        printf("retrieval of this type is not supported\n");
      }
    }

  } else {
    acpString msg(m_name);
    msg += " is not a readable property";
    pShell->badInput(msg);
  }
}
