/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aRobotTest.c   					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a Windows stdio-based aRobot     */
/*              test program.                                      */
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


#include <stdio.h>

#include "aMemLeakDebug.h"

// leaked object checking
#ifdef aDEBUG
void* operator new(size_t size) {return aMemAlloc(size);}
void operator delete(void *p) throw() {return aMemFree(p);}
#endif /* aDEBUG */


#include "aIO.h"
#include "aStem.h"
#include "acpException.h"
#include "acpRobot.h"

#include "aCmd.tea"
#include "aRobotDefs.tea"

#include "aStream_STDIO_Console.h"

#include "acpList.h"
#include "acpValue.h"
#include "acpProperty.h"



aErr enumPropProc(
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS flags,
  void* vpRef);

aErr enumObjProc(
  acpObject& object,
  void* vpRef);

class acpExecCB :
  public acpCallback
{
  public:
	  		acpExecCB(
  			  acpRobot* pcRobot,
	  		  acpObject* pBehavior) :
	  		  m_pRobot(pcRobot),
  			  m_pBehavior(pBehavior) {}
    virtual aErr	call();

  protected:
    acpRobot*		m_pRobot;
    acpObject*		m_pBehavior;
};

aErr acpExecCB::call()
{
  int k;
  const char* pname;

  pname = m_pBehavior->getNamedValue("name")->getStringVal();
  k = m_pBehavior->getNamedValue("unique_id")->getIntVal();
  printf("*** EXEC-CB behavior \"%s\" with id %d executing...\n", pname, k);

  return aErrNone;
}

class acpCompCB :
  public acpCallback
{
  public:
	  		acpCompCB(
  			  acpRobot* pcRobot,
	  		  acpObject* pBehavior) :
	  		  m_pRobot(pcRobot),
  			  m_pBehavior(pBehavior) {}
    virtual aErr	call();

  protected:
    acpRobot*		m_pRobot;
    acpObject*		m_pBehavior;
};

aErr acpCompCB::call()
{
  printf("*** COMP-CB behavior \"%s\" completed with status = %d\n", 
  	 m_pBehavior->getNamedValue("name")->getStringVal(),
  	 m_pBehavior->getNamedValue("completion_status")->getIntVal());

  return aErrNone;
}



/////////////////////////////////////////////////////////////////////

class acpGarciaAppExecute :
  public acpCallback
{
  public:
	  		acpGarciaAppExecute(
  			  acpRobot* pcGarcia,
  			  acpObject* pBehavior
  			) :
  			  m_pcGarcia(pcGarcia),
  			  m_pBehavior(pBehavior) 
  			{}

    aErr		call();

  private:
    acpRobot*	 	m_pcGarcia;
    acpObject*		m_pBehavior;
};


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr acpGarciaAppExecute::call()
{
  printf("behavior \"%s\" with id %d executing...\n", 
  	 m_pBehavior->getNamedValue("name")->getStringVal(),
  	 m_pBehavior->getNamedValue("unique_id")->getIntVal());

  return aErrNone;

} /* sExecute callback*/



/////////////////////////////////////////////////////////////////////

class acpGarciaAppComplete :
  public acpCallback
{
  public:
	  		acpGarciaAppComplete(
  			  acpRobot* pcGarcia,
  			  acpObject* pBehavior
  			) :
  			  m_pcGarcia(pcGarcia),
  			  m_pBehavior(pBehavior) 
  			{}

    aErr		call();
    
  private:
    acpRobot*	 	m_pcGarcia;
    acpObject*		m_pBehavior;
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr acpGarciaAppComplete::call()
{
  printf("completed with status = %d\n", 
  	 m_pBehavior->getNamedValue("completion_status")->getIntVal());

  return aErrNone;

} /* sComplete callback*/



aErr enumPropProc(
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS flags,
  void* vpRef)
{
  printf("%s%2i %04X %s\n", (char*)vpRef, nIndex, flags, pName);
  return aErrNone;
}

aErr enumObjProc(
  acpObject& object,
  void* vpRef)
{
  acpValue* pval;
  pval = object.getNamedValue("classname");
  if (pval)
    printf("CLASS: %s\n", pval->getStringVal());
  pval = object.getNamedValue("name");
  if (pval)
    printf("NAME: %s\n", pval->getStringVal());
  object.enumProperties(enumPropProc,(void*)"  ");
  return aErrNone;
}


void do_dump(acpRobot* pRobot, aIOLib ioRef);
void do_multi_prop(acpRobot* pRobot, aIOLib ioRef);
void do_garcia_app(acpRobot* pRobot, aIOLib ioRef);
void do_garcia_prim(acpRobot* pRobot, aIOLib ioRef);
void do_garcia_prop(acpRobot* pRobot, aIOLib ioRef);
void do_garcia_obj(acpRobot* pRobot, aIOLib ioRef);
void do_stem_object(acpRobot* pRobot, aIOLib ioRef);
void do_object(acpRobot* pRobot, aIOLib ioRef);
void do_property(acpRobot* pRobot, aIOLib ioRef);
void do_action(acpRobot* pRobot, aIOLib ioRef);
void do_script(acpRobot* pRobot, aIOLib ioRef);
void do_lrv(acpRobot* pRobot, aIOLib ioRef);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_lrv
 */

void do_lrv(acpRobot* pRobot, aIOLib ioRef)
{
//  aErr err = aErrNone;
  acpObject* pBehavior;
  int maxspeed = 16000;

  acpValue vcbe;
  acpValue vcbc;
  acpValue vL(0);
  acpValue vR(0);
  
  printf("LRV test\n");

  pBehavior = pRobot->createNamedBehavior("null", "drive");
  vcbe.set(new acpExecCB(pRobot, pBehavior));
  vcbc.set(new acpCompCB(pRobot, pBehavior));
  pBehavior->setNamedValue("execute_callback", &vcbe); 
  pBehavior->setNamedValue("completion_callback", &vcbc);

  pRobot->queueBehavior(pBehavior);
/*
  for (int i = 0; i < 30; i++) {
    pRobot->handleCallbacks(100);
  }
*/
  for (int i = 1000; i < maxspeed; i+=1000) {
    vL.set(i);
    vR.set(i);
    pRobot->setNamedValue("speed-left", &vL);
    pRobot->setNamedValue("speed-right", &vR);
    pRobot->handleCallbacks(100);
  }
  for (int i = maxspeed; i >= 0; i-=1000) {
    vL.set(i);
    vR.set(i);
    pRobot->setNamedValue("speed-left", &vL);
    pRobot->setNamedValue("speed-right", &vR);
    pRobot->handleCallbacks(100);
  }
  pRobot->flushQueuedBehaviors();

}




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_script
 */

void do_script(acpRobot* pRobot, aIOLib ioRef)
{
  aErr err = aErrNone;
  aStreamRef status;
  acpValue vcbe;
  acpValue vcbc;
  acpObject* pBehavior;

  err = aStream_Create_STDIO_Console_Output(ioRef, &status);

  printf("script test\n");

  acpValue vstreamptr((void*)status);
  pRobot->setNamedValue("status_stream", &vstreamptr);

  pBehavior = pRobot->createNamedBehavior("script", "snuh");
  acpValue vname("foo.xml");
  vcbe.set(new acpExecCB(pRobot, pBehavior));
  vcbc.set(new acpCompCB(pRobot, pBehavior));
  pBehavior->setNamedValue("execute_callback", &vcbe); 
  pBehavior->setNamedValue("completion_callback", &vcbc);
  pBehavior->setNamedValue("filename", &vname);
  pRobot->queueBehavior(pBehavior);
/*
  pBehavior = pRobot->createNamedBehavior("global", "duh");
  acpValue vi((int)1);
  pBehavior->setNamedValue("user-led", &vi);
  vcbe.set(new acpExecCB(pRobot, pBehavior));
  vcbc.set(new acpCompCB(pRobot, pBehavior));
  pBehavior->setNamedValue("execute_callback", &vcbe); 
  pBehavior->setNamedValue("completion_callback", &vcbc);
  pRobot->queueBehavior(pBehavior);
*/
  for (int k = 0; k < 10; k++) {
    pRobot->handleCallbacks(100);
  }
  pRobot->flushQueuedBehaviors();
  for (int k = 0; k < 200; k++) {
    pRobot->handleCallbacks(100);
  }

  aStream_Destroy(ioRef, status, &err);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_action
 */

void do_action(acpRobot* pRobot, aIOLib ioRef)
{
  aErr err = aErrNone;
  aStreamRef status;
  acpObject* pBehavior;
//  int i;

  acpValue vfloat(0.0f);
  acpValue vint(0);
  
  err = aStream_Create_STDIO_Console_Output(ioRef, &status);

  printf("action test\n");

  acpValue vstreamptr((void*)status);
  pRobot->setNamedValue("status_stream", &vstreamptr);
  
  acpValue vex1(0x0101);
  acpValue vex2(0x0102);

  acpValue vcbe;
  acpValue vcbc;

  pBehavior = pRobot->createNamedBehavior("sleep", "z1");
  vint.set(4000);
  pBehavior->setNamedValue("duration", &vint);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("move", "x");
  vcbe.set(new acpExecCB(pRobot, pBehavior));
  vcbc.set(new acpCompCB(pRobot, pBehavior));
  vfloat.set(0.25f);
  pBehavior->setNamedValue("distance", &vfloat);
  pBehavior->setNamedValue("expected-status", &vex1);
  pBehavior->setNamedValue("expected-status", &vex2);
  pBehavior->setNamedValue("execute_callback", &vcbe); 
  pBehavior->setNamedValue("completion_callback", &vcbc);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("sleep", "z2");
  vint.set(2000);
  pBehavior->setNamedValue("duration", &vint);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("move", "y");
  vcbe.set(new acpExecCB(pRobot, pBehavior));
  vcbc.set(new acpCompCB(pRobot, pBehavior));
  vfloat.set(0.5f);
  pBehavior->setNamedValue("distance", &vfloat);
  pBehavior->setNamedValue("expected-status", &vex1);
  pBehavior->setNamedValue("expected-status", &vex2);
  pBehavior->setNamedValue("execute_callback", &vcbe); 
  pBehavior->setNamedValue("completion_callback", &vcbc);
  pRobot->queueBehavior(pBehavior);

  if (err != aErrNone) {
    printf("An error occurred.\n");
  } else {
    printf("Queued okay...\n");

  for (int k = 0; k < 10; k++) {
    pRobot->handleCallbacks(100);
  }
//  pRobot->flushQueuedBehaviors();

  for (int i = 0; i < 100; i++) {
    float foo = 0.0f;
    acpValue* p;
    p = pRobot->getNamedValue("analog-0");
    foo = p->getFloatVal();
    printf("%10.5f\n", foo);
    pRobot->handleCallbacks(100);
  }

    // this loop checks to see when the list above is complete
    // and checks for callbacks in 100 mSec intervals  
//    while (!pRobot->getNamedValue("idle")->getBoolVal())
//      pRobot->handleCallbacks(100);
  }

//  for (i = 0; i < 10; i++)
//    pRobot->handleCallbacks(100);

//  aIO_MSSleep(ioRef, 500, NULL);  
  aStream_Destroy(ioRef, status, &err);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_property
 */

void do_property(acpRobot* pRobot, aIOLib ioRef)
{
  int i;

  printf("API TEA property tests\n");

  for (i = 0; i < 4; i++) {
    acpValue p;
    int b;
    p.set((int)(i % 2));
    pRobot->setNamedValue("user-led", &p);
    pRobot->handleCallbacks(100);
    b = pRobot->getNamedValue("user-led")->getIntVal();
    printf("%i\n", (int)b);
    pRobot->handleCallbacks(500);
  }

  for (i = 0; i < 20; i++) {
    float foo = 0.0f;
    acpValue* p;
    p = pRobot->getNamedValue("analog-0");
    foo = p->getFloatVal();
    printf("%10.5f\n", foo);
    pRobot->handleCallbacks(100);
  }
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_object
 */

void do_object(acpRobot* pRobot, aIOLib ioRef)
{
  int i;

  printf("API TEA user object tests\n");

  acpObject* pSonar;
  acpValue v;

  pSonar = pRobot->getSubObject("user_object", "sonar_0");
  
//  v.set((int)0xE2);
//  pSonar->setNamedValue("address", &v);
  i = pSonar->getNamedValue("address")->getIntVal();
  printf("address = %i\n", i);

//  v.set((int)0x06);
//  pSonar->setNamedValue("module", &v);
  i = pSonar->getNamedValue("module")->getIntVal();
  printf("module = %i\n", i);

  for (i = 0; i < 100; i++) {
    int n = pSonar->getNamedValue("light-level")->getIntVal();
    float r = pSonar->getNamedValue("range")->getFloatVal();
    printf("light = %3i,  range = %10.4f\n", n, r);
  }
  v.set(aROBOT_UNITS_INCHES);
  pRobot->setNamedValue("distance_units", &v);
  for (i = 0; i < 100; i++) {
    int n = pSonar->getNamedValue("light-level")->getIntVal();
    float r = pSonar->getNamedValue("range")->getFloatVal();
    printf("light = %3i,  range = %10.4f\n", n, r);
  }
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_stem_object
 */

void do_stem_object(acpRobot* pRobot, aIOLib ioRef)
{
  int i;
  int na;
  int nm;

  printf("STEM API TEA user object tests\n");
  
  acpObject* pA2D0;
  pA2D0 = pRobot->getSubObject("user_object", "a2d_0");

  for (i = 0; i < 5; i++) {
    int cfg;
    char s[10];
    acpValue v;
    strcpy(s, "dig_0");
    acpObject* pDig;
    s[4] = (char)('0' + i);
    pDig = pRobot->getSubObject("user_object", s);
    na = pDig->getNamedValue("id")->getIntVal();
    nm = pDig->getNamedValue("module")->getIntVal();
    v.set((int)0);
    pDig->setNamedValue("config", &v);
    void* p;
    p = pDig->getNamedValue("dataptr")->getVoidPtrVal();
    cfg = pDig->getNamedValue("config")->getIntVal();
    v.set((int)1);
    pDig->setNamedValue("value", &v);
    printf("%i %i %i %08X\n", na, nm, cfg, (int)(long)p);
  }

  for (i = 0; i < 4; i++) {
    int lmts;
    int cfg;
    float fpos;
    char s[10];
    acpValue v;
    strcpy(s, "srv_0");
    acpObject* pObj;
    s[4] = (char)('0' + i);
    pObj = pRobot->getSubObject("user_object", s);
    na = pObj->getNamedValue("id")->getIntVal();
    nm = pObj->getNamedValue("module")->getIntVal();
    v.set(-0.33f);
    pObj->setNamedValue("relpos", &v);
    void* p;
    p = pObj->getNamedValue("dataptr")->getVoidPtrVal();
    fpos = pObj->getNamedValue("abspos")->getFloatVal();
    lmts = pObj->getNamedValue("limits")->getIntVal();
    cfg = pObj->getNamedValue("config")->getIntVal();
    printf("%i %i %10.4f %04X %02X %08X\n", na, nm, fpos, lmts, cfg, 
	   (int)(long)p);
  }

  for (i = 0; i < 50; i++) {
    float v;
    na = pA2D0->getNamedValue("id")->getIntVal();
    nm = pA2D0->getNamedValue("module")->getIntVal();
    v = pA2D0->getNamedValue("value")->getFloatVal();
    printf("%i %i %10.4f\n", na, nm, v);
    pRobot->handleCallbacks(100);
  }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_garcia_obj
 */

void do_garcia_obj(acpRobot* pRobot, aIOLib ioRef)
{
  acpValue v;
  acpObject* pob = NULL;
  char* buff;

  printf("GARCIA API TEA object tests\n");

  pob = pRobot->getSubObject("user_object", "sp03");

  buff = (char*)pob->getNamedValue("dataptr")->getVoidPtrVal();
  aStringCopy(buff, "this is a test of my user object");

  v.set((int)aStringLen(buff));
  pob->setNamedValue("say-string", &v);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_garcia_prop
 */

void do_garcia_prop(acpRobot* pRobot, aIOLib ioRef)
{
  int i;
  int k;
  unsigned int uk;
  int n1, n2, n3, n4;
  int nn1, nn2, nn3;
  float f;
  float f1, f2, f3, f4, f5, f6;
  float fdl, fdr;
  acpValue v;

  printf("GARCIA API TEA property tests\n");

if (0) {
  n1 = pRobot->getNamedValue("user-flags")->getIntVal();
  n2 = pRobot->getNamedValue("stall-threshold")->getIntVal();
  n3 = pRobot->getNamedValue("stall-queue-size")->getIntVal();
  printf("%02X %2i %2i\n", n1, n2, n3);
  v.set((int)0x33);
  pRobot->setNamedValue("user-flags", &v);
  pRobot->setNamedValue("stall-threshold", &v);
  v.set((int)11);
  pRobot->setNamedValue("stall-queue-size", &v);
  nn1 = pRobot->getNamedValue("user-flags")->getIntVal();
  nn2 = pRobot->getNamedValue("stall-threshold")->getIntVal();
  nn3 = pRobot->getNamedValue("stall-queue-size")->getIntVal();
  printf("%02X %2i %2i\n", nn1, nn2, nn3);
  v.set(n1);
  pRobot->setNamedValue("user-flags", &v);
  v.set(n2);
  pRobot->setNamedValue("stall-threshold", &v);
  v.set(n3);
  pRobot->setNamedValue("stall-queue-size", &v);
  n1 = pRobot->getNamedValue("user-flags")->getIntVal();
  n2 = pRobot->getNamedValue("stall-threshold")->getIntVal();
  n3 = pRobot->getNamedValue("stall-queue-size")->getIntVal();
  printf("%02X %2i %2i\n", n1, n2, n3);
}

if (0) {
  v.set((int)0xFFCC);
  pRobot->setNamedValue("ir-receive", &v);
  for (i = 0; i < 100; i++) {
    k = pRobot->getNamedValue("user-button")->getIntVal();
    uk = pRobot->getNamedValue("ir-receive")->getIntVal();
    f = pRobot->getNamedValue("battery-voltage")->getFloatVal();
    printf("%3i %i %10.4f %04X\n", i, k, f, uk);
    pRobot->handleCallbacks(100);
  }
}

if (0) {
  for (i = 0; i < 10; i++) {
    v.set((int)1);
    pRobot->setNamedValue("user-led", &v);
    pRobot->setNamedValue("down-ranger-enable", &v);
    pRobot->setNamedValue("side-ranger-enable", &v);
    pRobot->setNamedValue("rear-ranger-enable", &v);
    pRobot->setNamedValue("front-ranger-enable", &v);
    pRobot->handleCallbacks(50);
    k = pRobot->getNamedValue("user-led")->getIntVal();
    n1 = pRobot->getNamedValue("down-ranger-enable")->getIntVal();
    n2 = pRobot->getNamedValue("side-ranger-enable")->getIntVal();
    n3 = pRobot->getNamedValue("rear-ranger-enable")->getIntVal();
    n4 = pRobot->getNamedValue("front-ranger-enable")->getIntVal();
    printf("%i %i %i %i %i\n", k, n1, n2, n3, n4);
    pRobot->handleCallbacks(200);
    v.set((int)0);
    pRobot->setNamedValue("user-led", &v);
    pRobot->setNamedValue("down-ranger-enable", &v);
    pRobot->setNamedValue("side-ranger-enable", &v);
    pRobot->setNamedValue("rear-ranger-enable", &v);
    pRobot->setNamedValue("front-ranger-enable", &v);
    pRobot->handleCallbacks(50);
    k = pRobot->getNamedValue("user-led")->getIntVal();
    n1 = pRobot->getNamedValue("down-ranger-enable")->getIntVal();
    n2 = pRobot->getNamedValue("side-ranger-enable")->getIntVal();
    n3 = pRobot->getNamedValue("rear-ranger-enable")->getIntVal();
    n4 = pRobot->getNamedValue("front-ranger-enable")->getIntVal();
    printf("%i %i %i %i %i\n", k, n1, n2, n3, n4);
    pRobot->handleCallbacks(200);
  }
}

if (0) {
  v.set((int)aROBOT_UNITS_METERS);
  pRobot->setNamedValue("distance_units", &v);
  v.set((float)1.5f);
  pRobot->setNamedValue("distance-left", &v);
  v.set((float)2.5f);
  pRobot->setNamedValue("distance-right", &v);
  v.set((float)0.1f);
  pRobot->setNamedValue("damped-speed-left", &v);
  v.set((float)0.2f);
  pRobot->setNamedValue("damped-speed-right", &v);
  v.set((int)aROBOT_UNITS_FEET);
  pRobot->setNamedValue("distance_units", &v);
  fdl = pRobot->getNamedValue("distance-left")->getFloatVal();
  fdr = pRobot->getNamedValue("distance-right")->getFloatVal();
  printf("%10.4f %10.4f\n", fdl, fdr);
  fdl = pRobot->getNamedValue("damped-speed-left")->getFloatVal();
  fdr = pRobot->getNamedValue("damped-speed-right")->getFloatVal();
  printf("%10.4f %10.4f\n", fdl, fdr);
}

if (0) {
  for (i = 0; i < 100; i++) {
    n1 = pRobot->getNamedValue("down-ranger-left")->getIntVal();
    n2 = pRobot->getNamedValue("down-ranger-right")->getIntVal();
    printf("%i %i\n", n1, n2);
    pRobot->handleCallbacks(100);
  }
}

if (0) {
/*
  float fVal;
  unsigned long t1;
  unsigned long t2;
   aIO_GetMSTicks(ioRef, &t1, NULL);
  for (i = 0; i < 100; i++) {
    fVal = pRobot->getNamedValue("front-ranger-right")->getFloatVal();
    printf("FR=%5.3f  ", fVal);
    printf("\n");
//    pRobot->handleCallbacks(1);
  }
   aIO_GetMSTicks(ioRef, &t2, NULL);
     printf("%i\n", (int)(t2 - t1));
*/
  v.set((int)1);
  pRobot->setNamedValue("down-ranger-enable", &v);
  pRobot->setNamedValue("side-ranger-enable", &v);
  pRobot->setNamedValue("rear-ranger-enable", &v);
  pRobot->setNamedValue("front-ranger-enable", &v);
  for (i = 0; i < 200; i++) {
    f1 = pRobot->getNamedValue("front-ranger-left")->getFloatVal();
    f2 = pRobot->getNamedValue("side-ranger-left")->getFloatVal();
    f3 = pRobot->getNamedValue("rear-ranger-left")->getFloatVal();
    f4 = pRobot->getNamedValue("front-ranger-right")->getFloatVal();
    f5 = pRobot->getNamedValue("side-ranger-right")->getFloatVal();
    f6 = pRobot->getNamedValue("rear-ranger-right")->getFloatVal();
    printf("fl=%8.4f sl=%8.4f rl=%8.4f   fr=%8.4f sr=%8.4f rr=%8.4f\n", f1,f2,f3,f4,f5,f6);
    pRobot->handleCallbacks(1);
  }
}

if (1) {
  f1 = pRobot->getNamedValue("front-ranger-threshold")->getFloatVal();
  f2 = pRobot->getNamedValue("side-ranger-threshold")->getFloatVal();
  f3 = pRobot->getNamedValue("rear-ranger-threshold")->getFloatVal();
  printf("ft=%8.4f st=%8.4f rt=%8.4f\n", f1,f2,f3);
  v.set((float)0.3);
  pRobot->setNamedValue("front-ranger-threshold", &v);
  v.set((float)0.4);
  pRobot->setNamedValue("side-ranger-threshold", &v);
  v.set((float)0.5);
  pRobot->setNamedValue("rear-ranger-threshold", &v);
    pRobot->handleCallbacks(200);
  f4 = pRobot->getNamedValue("front-ranger-threshold")->getFloatVal();
  f5 = pRobot->getNamedValue("side-ranger-threshold")->getFloatVal();
  f6 = pRobot->getNamedValue("rear-ranger-threshold")->getFloatVal();
  printf("ft=%8.4f st=%8.4f rt=%8.4f\n", f4,f5,f6);
  v.set(f4);
  pRobot->setNamedValue("front-ranger-threshold", &v);
  v.set(f5);
  pRobot->setNamedValue("side-ranger-threshold", &v);
  v.set(f6);
  pRobot->setNamedValue("rear-ranger-threshold", &v);
    pRobot->handleCallbacks(200);
  f4 = pRobot->getNamedValue("front-ranger-threshold")->getFloatVal();
  f5 = pRobot->getNamedValue("side-ranger-threshold")->getFloatVal();
  f6 = pRobot->getNamedValue("rear-ranger-threshold")->getFloatVal();
  printf("ft=%8.4f st=%8.4f rt=%8.4f\n", f1,f2,f3);
}

if (0) {
  for (i = 0; i < 16; i++) {
    v.set((int)i);
    pRobot->setNamedValue("ir-transmit", &v);
    pRobot->handleCallbacks(250);
  }
}

  for (i = 0; i < 5; i++) {
    pRobot->handleCallbacks(100);
  }
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_garcia_prim
 */

void do_garcia_prim(acpRobot* pRobot, aIOLib ioRef)
{
  aErr err = aErrNone;
  aStreamRef status;
  acpObject* pBehavior;

  printf("GARCIA API TEA primitive tests\n");

  acpValue vfloat(0.0f);
  acpValue vint(0);
  
  err = aStream_Create_STDIO_Console_Output(ioRef, &status);

  acpValue vstreamptr((void*)status);
  pRobot->setNamedValue("status_stream", &vstreamptr);
  
  acpValue vex(0x0101);

  acpValue vcbe;
  acpValue vcbc;

  pBehavior = pRobot->createNamedBehavior("sleep", "z1");
  vint.set(4000);
  pBehavior->setNamedValue("duration", &vint);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("move", "x");
  vcbe.set(new acpExecCB(pRobot, pBehavior));
  vcbc.set(new acpCompCB(pRobot, pBehavior));
  vfloat.set(0.25f);
  pBehavior->setNamedValue("distance", &vfloat);
  pBehavior->setNamedValue("expected-status", &vex);
  pBehavior->setNamedValue("execute_callback", &vcbe); 
  pBehavior->setNamedValue("completion_callback", &vcbc);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("sleep", "z2");
  vint.set(2000);
  pBehavior->setNamedValue("duration", &vint);
  pRobot->queueBehavior(pBehavior);

  if (err != aErrNone) {
    printf("An error occurred.\n");
  } else {
    printf("Queued okay...\n");

    for (int k = 0; k < 10; k++) {
      pRobot->handleCallbacks(100);
    }
//  pRobot->flushQueuedBehaviors();
/*
  for (int i = 0; i < 100; i++) {
    float foo = 0.0;
    acpValue* p;
    p = pRobot->getNamedValue("analog-0");
    foo = p->getFloatVal();
    printf("%10.5f\n", foo);
    pRobot->handleCallbacks(100);
  }
*/
    // this loop checks to see when the list above is complete
    // and checks for callbacks in 100 mSec intervals  
    while (!pRobot->getNamedValue("idle")->getBoolVal())
      pRobot->handleCallbacks(100);
  }
printf("done\n");
//  for (int nn = 0; nn < 10; nn++)
//    pRobot->handleCallbacks(100);

  aIO_MSSleep(ioRef, 500, NULL);  
  aStream_Destroy(ioRef, status, &err);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_garcia_app
 */

void do_garcia_app(acpRobot* pGarcia, aIOLib ioRef)
{
  acpObject* pBehavior;

  printf("waiting for garcia\n");

  while (!pGarcia->getNamedValue("active")->getBoolVal()) {
    printf("still waiting\n");
    aIO_MSSleep(ioRef, 100, NULL);
  }

  printf("queueing a simple triangle\n");

  acpValue executeCB;
  acpValue completeCB;
  acpValue lengthVal(0.5f);
  acpValue rotationVal((float)(3.1459 * 2 / 3));
  pBehavior = pGarcia->createNamedBehavior("move", "out");
  pBehavior->setNamedValue("distance", &lengthVal);
  executeCB.set(new acpGarciaAppExecute(pGarcia, pBehavior));
  pBehavior->setNamedValue("execute_callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(pGarcia, pBehavior));
  pBehavior->setNamedValue("completion_callback", &completeCB);
  pGarcia->queueBehavior(pBehavior);

  pBehavior = pGarcia->createNamedBehavior("pivot", "pivot 1");
  pBehavior->setNamedValue("angle", &rotationVal);
  executeCB.set(new acpGarciaAppExecute(pGarcia, pBehavior));
  pBehavior->setNamedValue("execute_callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(pGarcia, pBehavior));
  pBehavior->setNamedValue("completion_callback", &completeCB);
  pGarcia->queueBehavior(pBehavior);

  pBehavior = pGarcia->createNamedBehavior("move", "over");
  pBehavior->setNamedValue("distance", &lengthVal);
  executeCB.set(new acpGarciaAppExecute(pGarcia, pBehavior));
  pBehavior->setNamedValue("execute_callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(pGarcia, pBehavior));
  pBehavior->setNamedValue("completion_callback", &completeCB);
  pGarcia->queueBehavior(pBehavior);

  // we don't set the callbacks on this behavior so there will
  // be no display information about this pivot behavior
  pBehavior = pGarcia->createNamedBehavior("pivot", "pivot 2");
  pBehavior->setNamedValue("angle", &rotationVal);
  pGarcia->queueBehavior(pBehavior);

  pBehavior = pGarcia->createNamedBehavior("move", "back");
  pBehavior->setNamedValue("distance", &lengthVal);
  executeCB.set(new acpGarciaAppExecute(pGarcia, pBehavior));
  pBehavior->setNamedValue("execute_callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(pGarcia, pBehavior));
  pBehavior->setNamedValue("completion_callback", &completeCB);
  pGarcia->queueBehavior(pBehavior);

  pBehavior = pGarcia->createNamedBehavior("pivot", "pivot 3");
  pBehavior->setNamedValue("angle", &rotationVal);
  executeCB.set(new acpGarciaAppExecute(pGarcia, pBehavior));
  pBehavior->setNamedValue("execute_callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(pGarcia, pBehavior));
  pBehavior->setNamedValue("completion_callback", &completeCB);
  pGarcia->queueBehavior(pBehavior);

  // this loop checks to see when the list above is complete
  // and checks for callbacks in 100 mSec intervals  
  while (!pGarcia->getNamedValue("idle")->getBoolVal())
    pGarcia->handleCallbacks(100);

  printf("done\n");

} /* main */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_mulit_prop
 */

void do_multi_prop(acpRobot* pRobot, aIOLib ioRef)
{
  int i;
  float f1 = 0.0f;
  float f2 = 0.0f;
  acpObject* pBehavior;
  acpValue v;

  printf("Robot API multi-stream tests\n");

  pBehavior = pRobot->createNamedBehavior("blink1", "b1");
  v.set(5000);
  pBehavior->setNamedValue("duration", &v);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("sleep", "delay1");
  v.set(500);
  pBehavior->setNamedValue("duration", &v);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("blink2", "b2");
  v.set(5000);
  pBehavior->setNamedValue("duration", &v);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("sleep", "delay2");
  v.set(500);
  pBehavior->setNamedValue("duration", &v);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("blink1", "b3");
  v.set(5000);
  pBehavior->setNamedValue("duration", &v);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("sleep", "delay3");
  v.set(500);
  pBehavior->setNamedValue("duration", &v);
  pRobot->queueBehavior(pBehavior);

  pBehavior = pRobot->createNamedBehavior("blink2", "b4");
  v.set(5000);
  pBehavior->setNamedValue("duration", &v);
  pRobot->queueBehavior(pBehavior);

  for (i = 0; i < 100; i++) {
    acpObject* pFoo = NULL;
    int n = 0;
    pFoo = pRobot->getSubObject("user_object", "link2_4");
    n = pFoo->getNamedValue("value")->getIntVal();
    f1 = pRobot->getNamedValue("a0_com1")->getFloatVal();
    f2 = pRobot->getNamedValue("a0_com2")->getFloatVal();
    printf("%10.4f %10.4f %i\n", f1, f2, n);
    pRobot->handleCallbacks(100);
  }
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_dump
 */

void do_dump(acpRobot* pRobot, aIOLib ioRef)
{
  int i;
  
  printf("%i properties\n", pRobot->numProperties());
  printf("%i subobjects\n", pRobot->numSubObjects());
  pRobot->enumProperties(enumPropProc, (void*)"");
  pRobot->enumSubObjects(enumObjProc, NULL);
  for (i = 0; i < 10; i++)
    pRobot->handleCallbacks(100);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * main
 */

int main(int argc, char* argv[])
{
  int i;
  aIOLib ioRef;
  acpRobot robot("garcia_api");

  aIO_GetLibRef(&ioRef, NULL);

  printf("waiting for robot\n");
  while (!robot.getNamedValue("active")->getBoolVal()) {
    printf("still waiting\n");
    aIO_MSSleep(ioRef, 100, NULL);
  }

  // tests
//  do_dump(&robot,ioRef);
//  do_multi_prop(&robot, ioRef);
//  do_garcia_app(&robot, ioRef);
//  do_garcia_prim(&robot, ioRef);
//  do_garcia_prop(&robot, ioRef);
//  do_garcia_obj(&robot, ioRef);
//  do_stem_object(&robot, ioRef);
  do_object(&robot, ioRef);
//  do_property(&robot, ioRef);
//  do_action(&robot, ioRef);
//  do_script(&robot, ioRef);
//  do_lrv(&robot, ioRef);

  printf("idling...\n");

  // idle for a while
  for (i = 0; i < 50; i++)
    robot.handleCallbacks(100);

  printf("done\n");

  aIO_ReleaseLibRef(ioRef, NULL);
  aLeakCheckCleanup();

} /* main */
