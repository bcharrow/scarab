/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aGarciaTest.c   					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a Windows stdio-based Garcia     */
/*              demo program.                                      */
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
#include "acpGarcia.h"

#include "aCmd.tea"
#include "aGarciaDefs.tea"
#include "aGarciaGeom.h"
#include "acpBehavior.h"

#include "aStream_STDIO_Console.h"

#include "acpList.h"
#include "acpValue.h"
#include "acpProperty.h"

#include "acpGarciaServo.h"
#include "acpGarciaSonar.h"
#include "acpGarciaCamera.h"

void dumpservo(acpObject* pServo);
void jiggleservo(acpObject* pservo, aIOLib ioRef);
void steer(acpGarcia* pGarcia, float fmaxspd, float fminspd, float fdir);

aErr enumPropProc(
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS flags,
  void* vpRef);

aErr enumObjProc(
  acpObject& object,
  void* vpRef);

void do_intproptest(acpGarcia* pGarcia, aIOLib ioRef);
void do_fproptest(acpGarcia* pGarcia, aIOLib ioRef);
void do_one_triangle(acpGarcia* pGarcia, aIOLib ioRef);
void do_many_triangles(acpGarcia* pGarcia, aIOLib ioRef);
void do_arena(acpGarcia* pGarcia, aIOLib ioRef);
void do_some_io(acpGarcia* pGarcia, aIOLib ioRef);
void do_dock(acpGarcia* pGarcia, aIOLib ioRef);
void do_align(acpGarcia* pGarcia, aIOLib ioRef, char cside);

void do_outandback(acpGarcia* pGarcia, aIOLib ioRef);
void do_geomtest(acpGarcia* pGarcia, aIOLib ioRef);
void do_veltest(acpGarcia* pGarcia, aIOLib ioRef);
void do_rangers(acpGarcia* pGarcia, aIOLib ioRef);
void do_irtest(acpGarcia* pGarcia, aIOLib ioRef);
void do_action(acpGarcia* pGarcia, aIOLib ioRef);
void do_primitive(acpGarcia* pGarcia, aIOLib ioRef);
void do_servotest(acpGarcia* pGarcia, aIOLib ioRef);
void do_dump(acpGarcia* pGarcia, aIOLib ioRef);
void do_sonartest(acpGarcia* pGarcia, aIOLib ioRef);



class acpExecCB :
  public acpCallback
{
  public:
	  		acpExecCB(
  			  acpGarcia* pcGarcia,
	  		  acpObject* pBehavior) :
	  		  m_pGarcia(pcGarcia),
  			  m_pBehavior(pBehavior) {}
    virtual aErr	call();

  protected:
    acpGarcia*		m_pGarcia;
    acpObject*		m_pBehavior;
};

aErr acpExecCB::call()
{
  int k;
  const char* pname;

  pname = m_pBehavior->getNamedValue("name")->getStringVal();
  k = m_pBehavior->getNamedValue("unique-id")->getIntVal();
  printf("behavior \"%s\" with id %d executing...\n", pname, k);

  return aErrNone;
}

class acpCompCB :
  public acpCallback
{
  public:
	  		acpCompCB(
  			  acpGarcia* pcGarcia,
	  		  acpObject* pBehavior) :
	  		  m_pGarcia(pcGarcia),
  			  m_pBehavior(pBehavior) {}
    virtual aErr	call();

  protected:
    acpGarcia*		m_pGarcia;
    acpObject*		m_pBehavior;
};

aErr acpCompCB::call()
{
  printf("behavior \"%s\" completed with status = %d\n", 
  	 m_pBehavior->getNamedValue("name")->getStringVal(),
  	 m_pBehavior->getNamedValue("completion-status")->getIntVal());

  return aErrNone;
}



void steer(acpGarcia* pGarcia, float fmaxspd, float fminspd, float fdir)
{
  float flmt = fmaxspd - fminspd;
  if (fdir >= 0) {
    if (fdir > flmt) fdir = flmt;
    acpValue leftVal(fmaxspd);
    pGarcia->setNamedValue("damped-speed-left", &leftVal);
    acpValue rightVal(fmaxspd - fdir);
    pGarcia->setNamedValue("damped-speed-right", &rightVal);
  } else {
    fdir = -fdir;
    if (fdir > flmt) fdir = flmt;
    acpValue leftVal(fmaxspd - fdir);
    pGarcia->setNamedValue("damped-speed-left", &leftVal);
    acpValue rightVal(fmaxspd);
    pGarcia->setNamedValue("damped-speed-right", &rightVal);
  }
}

void dumpservo(acpObject* pServo)
{
  printf("abs pos         %12.8f\n", pServo->getNamedValue(aGSP_POSITION_ABS)->getFloatVal());
  printf("rel pos         %12.8f\n", pServo->getNamedValue(aGSP_POSITION_REL)->getFloatVal());
  printf("ena bit         %i\n",     pServo->getNamedValue(aGSP_ENABLE_FLAG)->getBoolVal());
  printf("inv bit         %i\n",     pServo->getNamedValue(aGSP_INVERT_FLAG)->getBoolVal());
  printf("disabled state  %i\n",     pServo->getNamedValue(aGSP_DISABLEDSTATE)->getBoolVal());
  printf("speed           %i\n",     pServo->getNamedValue(aGSP_SPEED)->getIntVal());
  printf("offset          0x%02X\n",   pServo->getNamedValue(aGSP_OFFSET)->getIntVal());
  printf("range           0x%02X\n",   pServo->getNamedValue(aGSP_RANGE)->getIntVal());
}

void jiggleservo(acpObject* pservo, aIOLib ioRef)
{
  int k;

  acpValue vfloat(0.5f);

  pservo->setNamedValue("position-abs", &vfloat);
  aIO_MSSleep(ioRef, 1000, NULL);
  for (k = 0; k < 2; k++)
  {
    dumpservo(pservo);
    vfloat.set(0.1f);
    pservo->setNamedValue("position-abs", &vfloat);
    aIO_MSSleep(ioRef, 1000, NULL);
    dumpservo(pservo);
    vfloat.set(0.9f);
    pservo->setNamedValue("position-abs", &vfloat);
    aIO_MSSleep(ioRef, 1000, NULL);
  }

  for (k = 0; k < 2; k++)
  {
    vfloat.set(0.5f);
    pservo->setNamedValue("position-abs", &vfloat);
    aIO_MSSleep(ioRef, 1000, NULL);
    dumpservo(pservo);
    vfloat.set(-0.4f);
    pservo->setNamedValue("position-rel", &vfloat);
    aIO_MSSleep(ioRef, 1000, NULL);
    dumpservo(pservo);
    vfloat.set(0.5f);
    pservo->setNamedValue("position-abs", &vfloat);
    aIO_MSSleep(ioRef, 1000, NULL);
    dumpservo(pservo);
    vfloat.set(+0.4f);
    pservo->setNamedValue("position-rel", &vfloat);
    aIO_MSSleep(ioRef, 1000, NULL);
    dumpservo(pservo);
  }
}

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



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_geomtest
 */
 
void do_geomtest(acpGarcia* pGarcia, aIOLib ioRef)
{
  float x;
  int i;
  int j;
  int n;
  
  for (n = 0; n < 3; n++)
  {
    printf("unit code: %i\n", n);
  
    for (i = 100; i < 550; i++)
    {
      x = aGarciaGeom_ScaleRangerData((unsigned short)i, kGarciaGeomFrontRanger, n);
      j = aGarciaGeom_RangerToRaw(x, kGarciaGeomFrontRanger, n);
      printf("%i %12.4f %i %i\n", i, x, j, j-i);
    }
    for (i = 100; i < 550; i++)
    {
      x = aGarciaGeom_ScaleRangerData((unsigned short)i, kGarciaGeomSideRanger, n);
      j = aGarciaGeom_RangerToRaw(x, kGarciaGeomSideRanger, n);
      printf("%i %12.4f %i %i\n", i, x, j, j-i);
    }
    for (i = 100; i < 550; i++)
    {
      x = aGarciaGeom_ScaleRangerData((unsigned short)i, kGarciaGeomRearRanger, n);
      j = aGarciaGeom_RangerToRaw(x, kGarciaGeomRearRanger, n);
      printf("%i %12.4f %i %i\n", i, x, j, j-i);
    }
  }

} /* do_geomtest */




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_veltest
 */
 
void do_veltest(acpGarcia* pGarcia, aIOLib ioRef)
{
  acpObject* pBehavior;
  acpValue vfloat(0.0f);

  printf("null task\n");

  pBehavior = pGarcia->createNamedBehavior("null", "out");
  vfloat.set(0.25f);
  pBehavior->setNamedValue("acceleration", &vfloat);
  pGarcia->queueBehavior(pBehavior);

  aIO_MSSleep(ioRef, 200, NULL);

  if (1 && pGarcia->getNamedValue("active")->getBoolVal()) {

    printf("forward\n");
    vfloat.set(0.15f);
    pGarcia->setNamedValue("damped-speed-left", &vfloat);
    pGarcia->setNamedValue("damped-speed-right", &vfloat);
    aIO_MSSleep(ioRef, 4000, NULL);

    printf("stopping\n");
    vfloat.set(0.0f);
    pGarcia->setNamedValue("damped-speed-left", &vfloat);
    pGarcia->setNamedValue("damped-speed-right", &vfloat);
    aIO_MSSleep(ioRef, 4000, NULL);

    printf("reverse\n");
    vfloat.set(-0.15f);
    pGarcia->setNamedValue("damped-speed-left", &vfloat);
    pGarcia->setNamedValue("damped-speed-right", &vfloat);
    aIO_MSSleep(ioRef, 4000, NULL);

    printf("stopping\n");
    vfloat.set(0.0f);
    pGarcia->setNamedValue("damped-speed-left", &vfloat);
    pGarcia->setNamedValue("damped-speed-right", &vfloat);
    aIO_MSSleep(ioRef, 4000, NULL);
  }

  if (1 && pGarcia->getNamedValue("active")->getBoolVal()) {

    float r = 0.0f;
    float err;

    printf("hugging\n");
    for (;;) {
      if (pGarcia->getNamedValue("idle")->getBoolVal()) break;
      r = pGarcia->getNamedValue("side-ranger-right")->getFloatVal();
      if (r < 0.0001f) {
        printf("no wall\n");
        r = 0.75f;
      } 
      err = (r - 0.275f) / 2.0f;
      printf("errSR=%7.3f\n", err);
      steer(pGarcia, 0.2f, 0.05f, err);
    
      aIO_MSSleep(ioRef, 50, NULL);
    }
  }

  printf("stopping\n");

  vfloat.set(0.0f);
  pGarcia->setNamedValue("damped-speed-left", &vfloat);
  pGarcia->setNamedValue("damped-speed-right", &vfloat);
  aIO_MSSleep(ioRef, 1000, NULL);
  
} /* do_veltest */



#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_intproptest
 */
 
void do_intproptest(acpGarcia* pGarcia, aIOLib ioRef)
{
  int j, k;
  for (k = 0; k < 64; k++)
  {
    if ((k % 8) == 0) pGarcia->setIntProperty("side-ranger-enable", 0);
    if ((k % 8) == 1) pGarcia->setIntProperty("rear-ranger-enable", 0);
    if ((k % 8) == 2) pGarcia->setIntProperty("front-ranger-enable", 0);
    if ((k % 8) == 3) pGarcia->setIntProperty("down-ranger-enable", 0);
    if ((k % 8) == 4) pGarcia->setIntProperty("side-ranger-enable", 1);
    if ((k % 8) == 5) pGarcia->setIntProperty("rear-ranger-enable", 1);
    if ((k % 8) == 6) pGarcia->setIntProperty("front-ranger-enable", 1);
    if ((k % 8) == 7) pGarcia->setIntProperty("down-ranger-enable", 1);
    pGarcia->setIntProperty("user-led", pGarcia->getIntProperty("user-button"));
    j = 0;
    j |= (pGarcia->getIntProperty("user-led") << 5);
    j |= (pGarcia->getIntProperty("user-button") << 4);
    j |= (pGarcia->getIntProperty("side-ranger-enable") << 3);
    j |= (pGarcia->getIntProperty("rear-ranger-enable") << 2);
    j |= (pGarcia->getIntProperty("front-ranger-enable") << 1);
    j |= (pGarcia->getIntProperty("down-ranger-enable"));
    printf("%i %02X\n",k%8,j);
    aIO_MSSleep(ioRef, 250, NULL);
  }
}
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_irtest
 */

void do_irtest(acpGarcia* pGarcia, aIOLib ioRef)
{
  int j, k;
  for (k = 0; k < 16; k++)
  {
    pGarcia->setIntProperty("ir-transmit", k);
    printf("TX %04X\n", k);
    aIO_MSSleep(ioRef, 250, NULL);
  }
  for (k = 0; k < 64; k++)
  {
    j = pGarcia->getIntProperty("ir-receive");
    printf("RX %04X\n", j);
    aIO_MSSleep(ioRef, 250, NULL);
  }
}
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_fproptest
 */
 
void do_fproptest(acpGarcia* pGarcia, aIOLib ioRef)
{
  int k;
  float bv;
  float bg;
  for (k = 0; k < 64; k++)
  {
    bv = pGarcia->getFloatProperty("battery-voltage");
    bg = pGarcia->getFloatProperty("battery-level");
    printf("%12.5f %12.5f\n", bv, bg);
    aIO_MSSleep(ioRef, 250, NULL);
  }
}
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_one_triangle
 */
 
void do_one_triangle(acpGarcia* pGarcia, aIOLib ioRef)
{
  aBehaviorRef task;

  printf("queueing a simple triangle\n");

  task = pGarcia->createBehavior("move", "out");
  pGarcia->setFloatProperty(task, "distance", 0.5f);
  pGarcia->queueBehavior(task);
  
  task = pGarcia->createBehavior("pivot", "turn");
  pGarcia->setFloatProperty(task, "angle", (float)(3.1459 * 2 / 3));
  pGarcia->queueBehavior(task);

  task = pGarcia->createBehavior("move", "over");
  pGarcia->setFloatProperty(task, "distance", 0.5f);
  pGarcia->queueBehavior(task);

  task = pGarcia->createBehavior("pivot", "turn");
  pGarcia->setFloatProperty(task, "angle", (float)(3.1459 * 2 / 3));
  pGarcia->queueBehavior(task);

  task = pGarcia->createBehavior("move", "back");
  pGarcia->setFloatProperty(task, "distance", 0.5f);
  pGarcia->queueBehavior(task);

  task = pGarcia->createBehavior("pivot", "turn");
  pGarcia->setFloatProperty(task, "angle", (float)(3.1459 * 2 / 3));
  pGarcia->queueBehavior(task);

  printf("waiting for triangle completion\n");
  while (!pGarcia->isIdle())
    aIO_MSSleep(ioRef, 100, NULL);
} 
#endif



#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_many_triangles
 */
 
void do_many_triangles(acpGarcia* pGarcia, aIOLib ioRef)
{

  aBehaviorRef task;

  try {

    float speed;

    for (speed = 0.2f; speed < 0.8f; speed += 0.2f) {

      printf("setting speed to %f\n", speed);  
      pGarcia->setFloatProperty("speed", speed);
      printf("validating speed...");
      float check = pGarcia->getFloatProperty("speed");
      float delta = check - speed;
      if ((delta < -0.0001) || (delta > 0.0001))
        printf("failed with %f\n", check);
      else
        printf("speed is good\n");

      task = pGarcia->createBehavior("move", "out");
      pGarcia->setFloatProperty(task, "distance", 0.5f);
      pGarcia->queueBehavior(task);

      task = pGarcia->createBehavior("pivot", "turn");
      pGarcia->setFloatProperty(task, "angle", (float)(3.1459 * 2 / 3));
      pGarcia->queueBehavior(task);
  
      task = pGarcia->createBehavior("say", "corner1");
      pGarcia->setStringProperty(task, "phrase", "corner 1");
      pGarcia->setFloatProperty(task, "volume", 1);
      pGarcia->setFloatProperty(task, "speed", speed);
      pGarcia->setFloatProperty(task, "pitch", 0.6f);
      pGarcia->queueBehavior(task);

      task = pGarcia->createBehavior("move", "over");
      pGarcia->setFloatProperty(task, "distance", 0.5f);
      pGarcia->queueBehavior(task);

      task = pGarcia->createBehavior("pivot", "turn");
      pGarcia->setFloatProperty(task, "angle", (float)(3.1459 * 2 / 3));
      pGarcia->queueBehavior(task);

      task = pGarcia->createBehavior("say", "corner2");
      pGarcia->setStringProperty(task, "phrase", "corner 2");
      pGarcia->setFloatProperty(task, "volume", 1);
      pGarcia->setFloatProperty(task, "speed", speed);
      pGarcia->setFloatProperty(task, "pitch", 0.6f);
      pGarcia->queueBehavior(task);

      task = pGarcia->createBehavior("move", "back");
      pGarcia->setFloatProperty(task, "distance", 0.5f);
      pGarcia->queueBehavior(task);

      task = pGarcia->createBehavior("pivot", "turn");
      pGarcia->setFloatProperty(task, "angle", (float)(3.1459 * 2 / 3));
      pGarcia->queueBehavior(task);

      task = pGarcia->createBehavior("say", "corner3");
      pGarcia->setStringProperty(task, "phrase", "corner 3");
      pGarcia->setFloatProperty(task, "volume", 1);
      pGarcia->setFloatProperty(task, "speed", speed);
      pGarcia->setFloatProperty(task, "pitch", 0.6f);
      pGarcia->queueBehavior(task);

      printf("waiting for triangle completion\n");
      while (!pGarcia->isIdle())
        aIO_MSSleep(ioRef, 100, NULL);
    }

  } catch (acpException e)
  {
    printf("exception\n");
  }

}
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_arena
 */
 
void do_arena(acpGarcia* pGarcia, aIOLib ioRef)
{
  aBehaviorRef task;
  float speed;
  int ledstate = 0;
  int laps = 0;

  printf("arena test\n");
  
  pGarcia->setIntProperty("down-ranger-enable",1);

  speed = 0.2f;  
      printf("setting speed to %f\n", speed);  
      pGarcia->setFloatProperty("speed", speed);
      printf("validating speed...");
      float check = pGarcia->getFloatProperty("speed");
      float delta = check - speed;
      if ((delta < -0.0001) || (delta > 0.0001))
        printf("failed with %f\n", check);
      else
        printf("speed is good\n");
  
  while (1)
  {
    ledstate = 1 - ledstate;
    pGarcia->setIntProperty("user-led",ledstate);
  
    task = pGarcia->createBehavior("hug", "firstwall");
    pGarcia->setIntProperty(task, "side", aGARCIA_MOTO_ARANGE_SIDE_RIGHT);
    pGarcia->setIntProperty(task, "mode", aGARCIA_HUGMODE_DEFAULT);
    pGarcia->setFloatProperty(task, "range", 0.275f);
    pGarcia->queueBehavior(task);

    task = pGarcia->createBehavior("pivot", "firstturn");
    pGarcia->setFloatProperty(task, "angle", (float)(3.1459 / 2));
    pGarcia->queueBehavior(task);

    task = pGarcia->createBehavior("hug", "secondwall");
    pGarcia->setIntProperty(task, "side", aGARCIA_MOTO_ARANGE_SIDE_RIGHT);
    pGarcia->setIntProperty(task, "mode", aGARCIA_HUGMODE_DEFAULT);
    pGarcia->setFloatProperty(task, "range", 0.275f);
    pGarcia->queueBehavior(task);

    task = pGarcia->createBehavior("pivot", "secondturn");
    pGarcia->setFloatProperty(task, "angle", (float)(3.1459 / 2));
    pGarcia->queueBehavior(task);

    task = pGarcia->createBehavior("hug", "thirdwall");
    pGarcia->setIntProperty(task, "side", aGARCIA_MOTO_ARANGE_SIDE_RIGHT);
    pGarcia->setIntProperty(task, "mode", aGARCIA_HUGMODE_DEFAULT);
    pGarcia->setFloatProperty(task, "range", 0.275f);
    pGarcia->queueBehavior(task);

    task = pGarcia->createBehavior("pivot", "thirdturn");
    pGarcia->setFloatProperty(task, "angle", (float)(3.1459 / 2));
    pGarcia->queueBehavior(task);

    task = pGarcia->createBehavior("hug", "fourthwall");
    pGarcia->setIntProperty(task, "side", aGARCIA_MOTO_ARANGE_SIDE_RIGHT);
    pGarcia->setIntProperty(task, "mode", aGARCIA_HUGMODE_DEFAULT);
    pGarcia->setFloatProperty(task, "range", 0.275f);
    pGarcia->queueBehavior(task);

    task = pGarcia->createBehavior("pivot", "fourthturn");
    pGarcia->setFloatProperty(task, "angle", (float)(3.1459 / 2));
    pGarcia->queueBehavior(task);

    printf("executing arena test\n");
    while (!pGarcia->isIdle())
      aIO_MSSleep(ioRef, 100, NULL);
    
    laps++;
    printf("laps completed:  %i\n", laps);
  }   
} 
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_some_io
 */
 
void do_some_io(acpGarcia* pGarcia, aIOLib ioRef)
{
  int ledstate = 0;
  int i;
  float fVal;

  printf("ranger test\n");

  // down rangers off
  pGarcia->setIntProperty("down-ranger-enable",aFalse);

  
   for (i = 0; i < 100; i++)
   {
     ledstate = 1 - ledstate;
     pGarcia->setIntProperty("user-led", ledstate);
     fVal = pGarcia->getFloatProperty("side-ranger-left");
     printf("SL=%7.3f  ", fVal);
     fVal = pGarcia->getFloatProperty("side-ranger-right");
     printf("SR=%7.3f  ", fVal);
     fVal = pGarcia->getFloatProperty("front-ranger-left");
     printf("FL=%7.3f  ", fVal);
     fVal = pGarcia->getFloatProperty("front-ranger-right");
     printf("FR=%7.3f  ", fVal);
     fVal = pGarcia->getFloatProperty("rear-ranger-left");
     printf("BL=%7.3f  ", fVal);
     fVal = pGarcia->getFloatProperty("rear-ranger-right");
     printf("BR=%7.3f", fVal);
     printf("\n");
     aIO_MSSleep(ioRef, 100, NULL);
   }
} 
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_dock
 */
 
void do_dock(acpGarcia* pGarcia, aIOLib ioRef)
{
  aBehaviorRef task;

  printf("docking test\n");

//  pGarcia->setFloatProperty("speed", 0.1f);

  task = pGarcia->createBehavior("global", "x1");
  pGarcia->setIntProperty(task, "user-led", 1);
  pGarcia->setIntProperty(task, "distance-units", aGARCIA_DISTANCE_METERS);
  pGarcia->setFloatProperty(task, "speed", 0.1f);
  pGarcia->queueBehavior(task);

  task = pGarcia->createBehavior("move", "x1");
  pGarcia->setFloatProperty(task, "distance", 0.275f);
  pGarcia->queueBehavior(task);

  task = pGarcia->createBehavior("move", "x2");
  pGarcia->setFloatProperty(task, "distance", 0.275f);
  pGarcia->setIntProperty(task, "expected-status", aGARCIA_ERRFLAG_FRONTR_LEFT);
  pGarcia->setIntProperty(task, "expected-status", aGARCIA_ERRFLAG_FRONTR_RIGHT);
  pGarcia->queueBehavior(task);

//  task = pGarcia->createBehavior("hug", "finddock");
//  pGarcia->setFloatProperty(task, "range", 0.275f); // 250 raw
//  pGarcia->setIntProperty(task, "mode", aGARCIA_HUGMODE_DOCKFIND);
//  pGarcia->setIntProperty(task, "side", aGARCIA_MOTO_ARANGE_SIDE_LEFT);
//  pGarcia->queueBehavior(task);

/*
  task = pGarcia->createBehavior("turn", "point");
  pGarcia->setFloatProperty(task, "angle", (float)(3.1459 * 1 / 2));
  pGarcia->setIntProperty(task, "side", aGARCIA_MOTO_MOTOR_LEFT);
  pGarcia->queueBehavior(task);

  task = pGarcia->createBehavior("dock", "charge");
  pGarcia->setFloatProperty(task, "range", 250.0f); // 250 raw
  pGarcia->queueBehavior(task);
*/
  printf("executing dock test\n");
  while (!pGarcia->isIdle())
    aIO_MSSleep(ioRef, 100, NULL);
} 
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_align
 */
 
void do_align(acpGarcia* pGarcia, aIOLib ioRef, char cside)
{
  aBehaviorRef task;

  printf("align test\n");

  task = pGarcia->createBehavior("align", "snuh");
  pGarcia->setFloatProperty(task, "range", 250.0f); // raw 250
  pGarcia->setIntProperty(task, "side", cside);
  pGarcia->queueBehavior(task);

  printf("executing align test\n");
  while (!pGarcia->isIdle())
    aIO_MSSleep(ioRef, 100, NULL);
} 
#endif


#if 0
void do_rangers(acpGarcia* pGarcia, aIOLib ioRef)
{
  int ledstate = 0;
  int i;
  float fVal;
//  int iVal;
  unsigned long t1;
  unsigned long t2;

  printf("ranger test\n");

  // down rangers off
//  pGarcia->setIntProperty("down-ranger-enable",aFalse);

   aIO_GetMSTicks(ioRef, &t1, NULL);
   for (i = 0; i < 100; i++)
   {
/*     ledstate = 1 - ledstate;
     fVal = pGarcia->getNamedValue("side-ranger-left")->getFloatVal();
     printf("SL=%5.3f  ", fVal);
     fVal = pGarcia->getNamedValue("side-ranger-right")->getFloatVal();
     printf("SR=%5.3f  ", fVal);
     fVal = pGarcia->getNamedValue("front-ranger-left")->getFloatVal();
     printf("FL=%5.3f  ", fVal);*/
     fVal = pGarcia->getNamedValue("front-ranger-right")->getFloatVal();
     printf("FR=%5.3f  ", fVal);
/*     fVal = pGarcia->getNamedValue("rear-ranger-left")->getFloatVal();
     printf("BL=%5.3f  ", fVal);
     fVal = pGarcia->getNamedValue("rear-ranger-right")->getFloatVal();
     printf("BR=%5.3f ", fVal);
     iVal = pGarcia->getNamedValue("down-ranger-left")->getIntVal();
     printf("DL=%i ", iVal);
     iVal = pGarcia->getNamedValue("down-ranger-right")->getIntVal();
     printf("DR=%i", iVal);*/
     printf("\n");
//     pGarcia->handleCallbacks(1);
//     aIO_MSSleep(ioRef, 100, NULL);
   }
   aIO_GetMSTicks(ioRef, &t2, NULL);
     printf("%i\n", t2 - t1);
} 
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_irtest
 */

void do_irtest(acpGarcia* pGarcia, aIOLib ioRef)
{
  int k;

  acpValue vInt(1);
  pGarcia->setNamedValue("user-led", &vInt);
  
  for (k = 0; k < 16; k++)
  {
    acpValue vInt(k);
    pGarcia->setNamedValue("ir-transmit", &vInt);
    printf("TX %04X\n", k);
    aIO_MSSleep(ioRef, 250, NULL);
  }
/*
  for (k = 0; k < 64; k++)
  {
    int j;
    j = pGarcia->getNamedValue("ir-receive")->getIntVal();
    printf("RX %04X\n", j);
    aIO_MSSleep(ioRef, 250, NULL);
  }
  */
}
#endif



#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_primitive
 */
 
void do_primitive(acpGarcia* pGarcia, aIOLib ioRef)
{
  acpObject* pBehavior;

  printf("primitive test\n");

  pBehavior = pGarcia->createNamedBehavior("say", "p3");
  acpValue phrase("q want to warm");
  pBehavior->setNamedValue("phrase", &phrase);
  acpValue speed(0.7f);
  pBehavior->setNamedValue("speed", &speed);
  acpValue pitch(0.6f);
  pBehavior->setNamedValue("pitch", &pitch);
  acpValue volume(1.0f);
  pBehavior->setNamedValue("volume", &volume);
  pGarcia->queueBehavior(pBehavior);

  printf("executing primitive test\n");

  // this loop checks to see when the list above is complete
  // and checks for callbacks in 100 mSec intervals  
  while (!pGarcia->getNamedValue("idle")->getBoolVal())
    pGarcia->handleCallbacks(100);
} 
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_action
 */

void do_action(acpGarcia* pGarcia, aIOLib ioRef)
{
  aErr err = aErrNone;
  aStreamRef status;
  acpObject* pBehavior;
  int i;

  acpValue vfloat(0.0f);
  acpValue vint(0);
  
  err = aStream_Create_STDIO_Console_Output(ioRef, &status);

  printf("script test\n");

  acpValue vstreamptr((void*)status);
  pGarcia->setNamedValue("status-stream", &vstreamptr);
  
  acpValue vex1(aGARCIA_ERRFLAG_FRONTR_RIGHT);
  acpValue vex2(aGARCIA_ERRFLAG_FRONTR_LEFT);

  acpValue executeCB;
  acpValue completeCB;

  pBehavior = pGarcia->createNamedBehavior("global", "x1");
  vint.set(1);
  vfloat.set(0.05f);
  pBehavior->setNamedValue("user-led", &vint);
  pBehavior->setNamedValue("speed", &vfloat);
  pGarcia->queueBehavior(pBehavior);

  pBehavior = pGarcia->createNamedBehavior("sleep", "naptime");
  vint.set(2000);
  pBehavior->setNamedValue("duration", &vint);
  pGarcia->queueBehavior(pBehavior);


  pBehavior = pGarcia->createNamedBehavior("global", "x2");
  vint.set(0);
  pBehavior->setNamedValue("user-led", &vint);
  pGarcia->queueBehavior(pBehavior);



  pBehavior = pGarcia->createNamedBehavior("move", "ox2");
  vfloat.set(0.2f);
  pBehavior->setNamedValue("distance", &vfloat);
  pBehavior->setNamedValue("expected-status", &vex1);
  pBehavior->setNamedValue("expected-status", &vex2);
  pGarcia->queueBehavior(pBehavior);

  pBehavior = pGarcia->createNamedBehavior("move", "ox3");
  vfloat.set(0.2f);
  pBehavior->setNamedValue("distance", &vfloat);
  pGarcia->queueBehavior(pBehavior);



  pBehavior = pGarcia->createNamedBehavior("move", "out 1");
  vfloat.set(-0.2f);
  executeCB.set(new acpExecCB(pGarcia, pBehavior));
  completeCB.set(new acpCompCB(pGarcia, pBehavior));
  pBehavior->setNamedValue("distance", &vfloat);
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  pBehavior->setNamedValue("completion-callback", &completeCB);
  pGarcia->queueBehavior(pBehavior);



  pBehavior = pGarcia->createNamedBehavior("script", "snuh");
  acpValue vname("oneside.xml");
  executeCB.set(new acpExecCB(pGarcia, pBehavior));
  completeCB.set(new acpCompCB(pGarcia, pBehavior));
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  pBehavior->setNamedValue("completion-callback", &completeCB);
  pBehavior->setNamedValue("filename", &vname);
  pGarcia->queueBehavior(pBehavior);


  pBehavior = pGarcia->createNamedBehavior("move", "out 2");
  vfloat.set(+0.25f);
  executeCB.set(new acpExecCB(pGarcia, pBehavior));
  completeCB.set(new acpCompCB(pGarcia, pBehavior));
  pBehavior->setNamedValue("distance", &vfloat);
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  pBehavior->setNamedValue("completion-callback", &completeCB);
  pGarcia->queueBehavior(pBehavior);


  if (err != aErrNone) {
    printf("An error occurred.\n");
  } else {
    printf("Queued okay...\n");
    // this loop checks to see when the list above is complete
    // and checks for callbacks in 100 mSec intervals  
    while (!pGarcia->getNamedValue("idle")->getBoolVal())
      pGarcia->handleCallbacks(100);
  }

  for (i = 0; i < 10; i++)
    pGarcia->handleCallbacks(100);

  aIO_MSSleep(ioRef, 500, NULL);  
  aStream_Destroy(ioRef, status, &err);
}
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_servotest
 */

void do_servotest(acpGarcia* pGarcia, aIOLib ioRef)
{
  acpObject* pservo = NULL;
  acpObject* pcamera = NULL;
  acpValue* pVal = NULL;

  pcamera = pGarcia->getSubObject("acpGarciaCamera", aGCP_CAMERA_BOOM);
  pservo = pGarcia->getSubObject("acpGarciaServo", aGSP_SERVO_0);

  acpValue v;

/*
  v.set((bool)1);
  pservo->setValue(aGSP_ENABLE_FLAG, &v);
  v.set(0);
  pservo->setValue(aGSP_SPEED, &v);
  dumpservo(pservo);
  jiggleservo(pservo, ioRef);
  
  v.set(15);
  pservo->setValue(aGSP_SPEED, &v);
  jiggleservo(pservo, ioRef);

  v.set(0);
  pservo->setValue(aGSP_OFFSET, &v);
  v.set(70);
  pservo->setValue(aGSP_RANGE, &v);
  jiggleservo(pservo, ioRef);
*/


  if (pcamera) {
    printf("testing pan and tilt...\n");
    v.set(0.9f);
    pcamera->setNamedValue("pan", &v);
    pcamera->setNamedValue("tilt", &v);
    aIO_MSSleep(ioRef, 1000, NULL);
    v.set(0.1f);
    pcamera->setNamedValue("pan", &v);
    pcamera->setNamedValue("tilt", &v);
    aIO_MSSleep(ioRef, 1000, NULL);
  }

}
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_outandback
 */
 
void do_outandback(acpGarcia* pGarcia, aIOLib ioRef)
{
  acpObject* pBehavior;
  float leftodometer;
  float rightodometer;

  printf("odometer test\n");
  
  acpValue v;
  
  v.set(0.2f);
  pGarcia->setNamedValue("speed", &v);

  pBehavior = pGarcia->createNamedBehavior("move", "out");
  v.set(0.85f);
  pBehavior->setNamedValue("distance", &v);
  pGarcia->queueBehavior(pBehavior);
  
  printf("waiting for OUT completion\n");
  while (!pGarcia->getNamedValue("idle")->getBoolVal())
    aIO_MSSleep(ioRef, 100, NULL);

  leftodometer = pGarcia->getNamedValue("distance-left")->getFloatVal();
  rightodometer = pGarcia->getNamedValue("distance-right")->getFloatVal();
  printf("L=%12.5f R=%12.5f\n",leftodometer,rightodometer);

  pBehavior = pGarcia->createNamedBehavior("move", "out");
  v.set(-0.85f);
  pBehavior->setNamedValue("distance", &v);
  pGarcia->queueBehavior(pBehavior);

  printf("waiting for BACK completion\n");
  while (!pGarcia->getNamedValue("idle")->getBoolVal())
    aIO_MSSleep(ioRef, 100, NULL);

  leftodometer = pGarcia->getNamedValue("distance-left")->getFloatVal();
  rightodometer = pGarcia->getNamedValue("distance-right")->getFloatVal();
  printf("L=%12.5f R=%12.5f\n",leftodometer,rightodometer);

  v.set(2.345f);
  pGarcia->setNamedValue("distance-left", &v);
  v.set(-2.345f);
  pGarcia->setNamedValue("distance-right", &v);

  leftodometer = pGarcia->getNamedValue("distance-left")->getFloatVal();
  rightodometer = pGarcia->getNamedValue("distance-right")->getFloatVal();
  printf("L=%12.5f R=%12.5f\n",leftodometer,rightodometer);
} 
#endif


#if 0
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_dump
 */

void do_dump(acpGarcia* pGarcia, aIOLib ioRef)
{
  int i;

  int sthr;
  int sqsize;  
  acpValue vstallthr(4);
  acpValue vstallqsize(11);
  
  sthr = pGarcia->getNamedValue("stall-threshold")->getIntVal();
  sqsize = pGarcia->getNamedValue("stall-queue-size")->getIntVal();
  printf("stall params = %i %i\n", sthr, sqsize);
  pGarcia->setNamedValue("stall-threshold", &vstallthr);
  pGarcia->setNamedValue("stall-queue-size", &vstallqsize);
  sthr = pGarcia->getNamedValue("stall-threshold")->getIntVal();
  sqsize = pGarcia->getNamedValue("stall-queue-size")->getIntVal();
  printf("stall params = %i %i\n", sthr, sqsize);
  
  printf("%i properties\n", pGarcia->numProperties());
  printf("%i subobjects\n", pGarcia->numSubObjects());
  pGarcia->enumProperties(enumPropProc, (void*)"");
  pGarcia->enumSubObjects(enumObjProc, NULL);
  for (i = 0; i < 100; i++)
    pGarcia->handleCallbacks(100);
}
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * do_sonartest
 */

void do_sonartest(acpGarcia* pGarcia, aIOLib ioRef)
{
  int i;
  acpValue vaddr(0xE2);
  acpObject* pobj = NULL;
  acpObject* psonar0 = NULL;
  acpObject* psonar1 = NULL;
  int n0, n1;
  float r0, r1;
  float q0, q1;
  char data[20];

  pobj = pGarcia->getSubObject("userobj", "userobj_0");

  if (pobj && 1) {

    int err;
    printf("serial relay test...\n");

    vaddr.set(4);
    pobj->setNamedValue("module", &vaddr);
    vaddr.set(3);
    pobj->setNamedValue("size", &vaddr);
    vaddr.set(0);
    pobj->setNamedValue("filter-byte", &vaddr);
    vaddr.set(0);
    pobj->setNamedValue("timeout", &vaddr);
    data[0]=18;
    data[1]=19;
    data[2]=1;
    vaddr.set((void*)data);
    pobj->setNamedValue("data-ptr", &vaddr);
    err = pobj->getNamedValue("result")->getIntVal();

    vaddr.set(4);
    pobj->setNamedValue("module", &vaddr);
    vaddr.set(3);
    pobj->setNamedValue("size", &vaddr);
    vaddr.set(0);
    pobj->setNamedValue("filter-byte", &vaddr);
    vaddr.set(0);
    pobj->setNamedValue("timeout", &vaddr);
    data[0]=18;
    data[1]=21;
    data[2]=1;
    vaddr.set((void*)data);
    pobj->setNamedValue("data-ptr", &vaddr);
    err = pobj->getNamedValue("result")->getIntVal();
    printf("switched to relay mode\n");
    
    pGarcia->handleCallbacks(1000);
    
    printf("setting relay stream index\n");
    vaddr.set(aGARCIA_MOTO_ADDR);
    pGarcia->setNamedValue("relay-index", &vaddr);

    printf("sending byte\n");
    vaddr.set(4);
    pobj->setNamedValue("module", &vaddr);
    vaddr.set(2);
    pobj->setNamedValue("size", &vaddr);
    vaddr.set(0);
    pobj->setNamedValue("filter-byte", &vaddr);
    vaddr.set(0);
    pobj->setNamedValue("timeout", &vaddr);
    data[0]=59;
    data[1]=65;
//    data[2]=1;
    vaddr.set((void*)data);
    pobj->setNamedValue("data-ptr", &vaddr);
    err = pobj->getNamedValue("result")->getIntVal();
    printf("completed\n");
    printf("error %i\n", err);
    
    int c;
    int k = 0;
    int foo = aErrNone;
    while (k < 12) {
      c = pGarcia->getNamedValue("relay-byte")->getIntVal();
      foo = pGarcia->getNamedValue("relay-status")->getIntVal();
      if (foo == aErrNone) {
        k++;
        printf("%i %i\n", c, foo);
      }
    }

    pGarcia->handleCallbacks(1000);

    vaddr.set(4);
    pobj->setNamedValue("module", &vaddr);
    vaddr.set(3);
    pobj->setNamedValue("size", &vaddr);
    vaddr.set(23);
    pobj->setNamedValue("filter-byte", &vaddr);
    vaddr.set(200);
    pobj->setNamedValue("timeout", &vaddr);
    data[0]=23;
    data[1]=1;
    data[2]=2;
    vaddr.set((void*)data);
    pobj->setNamedValue("data-ptr", &vaddr);
    err = pobj->getNamedValue("result")->getIntVal();
    printf("debug done\n");

    printf("error %i\n", err);
    pGarcia->handleCallbacks(3000);
  }



  if (pobj && 0) {

    int err;
    printf("user object found...\n");

    vaddr.set(2);
    pobj->setNamedValue("module", &vaddr);
    vaddr.set(2);
    pobj->setNamedValue("size", &vaddr);
    vaddr.set((unsigned char)0x82);
    pobj->setNamedValue("filter-byte", &vaddr);
    vaddr.set(1000);
    pobj->setNamedValue("timeout", &vaddr);
    data[0]=17;
    data[1]=9;
    vaddr.set((void*)data);
    pobj->setNamedValue("data-ptr", &vaddr);

    printf("%i\n", pobj->getNamedValue("module")->getIntVal());
    printf("%i\n", pobj->getNamedValue("size")->getIntVal());
    printf("0x%02X\n", 
	   (unsigned char)pobj->getNamedValue("filter-byte")->getIntVal());
    printf("%i\n", pobj->getNamedValue("timeout")->getIntVal());
    printf("%08X\n", 
	   (int)(long)pobj->getNamedValue("data-ptr")->getVoidPtrVal());

    err = pobj->getNamedValue("result")->getIntVal();

    for (i = 0; i < 100; i++)
      pGarcia->handleCallbacks(100);
  }

  psonar0 = pGarcia->getSubObject("sonar", aGRP_SONAR_0);
  psonar1 = pGarcia->getSubObject("sonar", aGRP_SONAR_1);

  if (psonar0 && psonar1 && 0) {
    printf("sonar found...\n");

    n0 = psonar0->getNamedValue("address")->getIntVal();
    n1 = psonar1->getNamedValue("address")->getIntVal();
    printf("verify addresses %02X %02X\n", n0, n1);

    for (i = 0; i < 100; i++) {
      q0 = psonar0->getNamedValue("light-value")->getFloatVal();
      r0 = psonar0->getNamedValue("range")->getFloatVal();
      q1 = psonar1->getNamedValue("light-value")->getFloatVal();
      r1 = psonar1->getNamedValue("range")->getFloatVal();
      printf("light = %8.4f %8.4f   range = %8.4f %8.4f\n", q0, q1, r0, r1);
      pGarcia->handleCallbacks(100);
    }
  }
  
}




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * main
 */

int main(int argc, char* argv[])
{
  aIOLib ioRef;
  acpGarcia garcia;

  aIO_GetLibRef(&ioRef, NULL);
  
  printf("waiting for garcia\n");
  while (!garcia.getNamedValue("active")->getBoolVal()) {
    printf("still waiting\n");
    aIO_MSSleep(ioRef, 100, NULL);
  }

  // give some time for application to get in sync with robot
  aIO_MSSleep(ioRef, 500, NULL);

// *** OLD TESTS THAT WILL NOT WORK ***
//  do_one_triangle(&garcia, ioRef);
//  do_arena(&garcia, ioRef);
//  do_some_io(&garcia, ioRef);
//  do_veltest(&garcia, ioRef);
//  do_outandback(&garcia, ioRef);
//  do_intproptest(&garcia, ioRef);
//  do_fproptest(&garcia, ioRef);
//  do_dock(&garcia, ioRef);

//  do_veltest(&garcia, ioRef);
//  do_irtest(&garcia, ioRef);
//  do_action(&garcia, ioRef);
//  do_geomtest(NULL, NULL);
//  do_primitive(&garcia, ioRef);
//  do_servotest(&garcia, ioRef);
//  do_outandback(&garcia, ioRef);
//  do_dump(&garcia, ioRef);
  do_sonartest(&garcia, ioRef);
//  do_rangers(&garcia, ioRef);

  
  printf("done\n");

  aIO_ReleaseLibRef(ioRef, NULL);
  
  aLeakCheckCleanup();

} /* main */
