/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: stdio_aRobotApp.cpp   					   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of a Windows stdio-based aRobot     //
//              example application.                               //
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


#include <stdio.h>

#include "aMemLeakDebug.h"

// leaked object checking
#ifdef aDEBUG
void* operator new(size_t size) {return aMemAlloc(size);}
void operator delete(void *p) throw() {return aMemFree(p);}
#endif // aDEBUG //


#include "aIO.h"
#include "acpRobot.h"
#include "aRobotDefs.tea"
#include "aStream_STDIO_Console.h"



/////////////////////////////////////////////////////////////////////
// enum proc calls

void handleCommands(const char* name, const char* task, aIOLib ioRef);
void do_dump(acpRobot* pRobot, aIOLib ioRef);
void do_garcia_demo(acpRobot* pRobot, aIOLib ioRef);
void do_stem_demo(acpRobot* pRobot, aIOLib ioRef);

aErr enumPropProc(
  const char* pName,
  const int nIndex,
  aPROPERTY_FLAGS flags,
  void* vpRef);

aErr enumObjProc(
  acpObject& object,
  void* vpRef);



/////////////////////////////////////////////////////////////////////
// enum proc calls

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

/////////////////////////////////////////////////////////////////////

aErr acpGarciaAppExecute::call()
{
  printf("behavior \"%s\" with id %d executing...\n", 
  	 m_pBehavior->getNamedValue("name")->getStringVal(),
  	 m_pBehavior->getNamedValue("unique_id")->getIntVal());

  return aErrNone;
}



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

/////////////////////////////////////////////////////////////////////

aErr acpGarciaAppComplete::call()
{
  printf("completed with status = %d\n",
    m_pBehavior->getNamedValue("completion_status")->getIntVal());

  return aErrNone;
}





/////////////////////////////////////////////////////////////////////
// do_stem_demo
//

void do_stem_demo(acpRobot* pRobot, aIOLib ioRef)
{
  int i;
  int nID;
  int nModule;

  printf("Beginning BrainStem GP Module API demo\n");

  acpValue v;
  acpObject* pObj;

  // A2D test
  // take samples from one input
  pObj = pRobot->getSubObject("user_object", "a2d_0");
  nID = pObj->getNamedValue("id")->getIntVal();
  nModule = pObj->getNamedValue("module")->getIntVal();
  for (i = 0; i < 50; i++) {
    float f = pObj->getNamedValue("value")->getFloatVal();
    printf("A2D[%i] from Module %i = %10.4f\n", nID, nModule, f);
    pRobot->handleCallbacks(100);
  }

  // digital IO tests
  // take samples from one input
  pObj = pRobot->getSubObject("user_object", "dig_0");
  nID = pObj->getNamedValue("id")->getIntVal();
  nModule = pObj->getNamedValue("module")->getIntVal();
  for (i = 0; i < 50; i++) {
    int n = pObj->getNamedValue("value")->getIntVal();
    printf("DIG[%i] from Module %i = %i\n", nID, nModule, n);
    pRobot->handleCallbacks(100);
  }

  // servo tests
  // jiggle a servo
  pObj = pRobot->getSubObject("user_object", "srv_0");
  nID = pObj->getNamedValue("id")->getIntVal();
  nModule = pObj->getNamedValue("module")->getIntVal();
  for (i = 0; i < 4; i++) {
    v.set(0.0f);
    pObj->setNamedValue("abspos", &v);
    pRobot->handleCallbacks(500);
    printf("moving to %10.4f\n", v.getFloatVal());
    v.set(1.0f);
    pObj->setNamedValue("abspos", &v);
    pRobot->handleCallbacks(500);
    printf("moving to %10.4f\n", v.getFloatVal());
  }
}



/////////////////////////////////////////////////////////////////////
// do_garcia_demo
//

void do_garcia_demo(acpRobot* pGarcia, aIOLib ioRef)
{
  acpValue v;
  acpObject* pBehavior;

  printf("Beginning Garcia API demo\n");

  // turn on user LED to show that robot is working
  v.set((int)1);
  pGarcia->setNamedValue("user-led", &v);

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

  // turn off user LED now that we are done
  v.set((int)0);
  pGarcia->setNamedValue("user-led", &v);

  // give time for any queued callbacks to be consumed
  pGarcia->handleCallbacks(1000);

  printf("done\n");

}



/////////////////////////////////////////////////////////////////////
// do_dump
//

void do_dump(acpRobot* pRobot, aIOLib ioRef)
{
  printf("%i properties\n", pRobot->numProperties());
  printf("%i subobjects\n", pRobot->numSubObjects());
  pRobot->enumProperties(enumPropProc, (void*)"");
  pRobot->enumSubObjects(enumObjProc, NULL);
}



/////////////////////////////////////////////////////////////////////
// handleCommands
//

void handleCommands(const char* name, const char* task, aIOLib ioRef)
{
  int ntype = 0;
  acpRobot robot(name);

  printf("Waiting for link...\n");
  while (!robot.getNamedValue("active")->getBoolVal()) {
    printf("Still waiting...\n");
    aIO_MSSleep(ioRef, 100, NULL);
  }

  if (!aStringCompare(name, "stem_api")) {
    ntype = 1;
  } else if (!aStringCompare(name, "garcia_api")) {
    ntype = 2;
  }

  if (!aStringCompare(task, "dump")) {

    // dump works with any type //
    do_dump(&robot, ioRef);

  } else if (!aStringCompare(task, "demo")) {

    // run a demo specific to API type
    switch (ntype) {
      case 1:
        do_stem_demo(&robot, ioRef);
        break;
      case 2:
        do_garcia_demo(&robot, ioRef);
        break;
      default:
        break;
    }
  }
}



/////////////////////////////////////////////////////////////////////
// main
//

int main(int argc, char* argv[])
{
  aIOLib ioRef;

  aIO_GetLibRef(&ioRef, NULL);

  if (argc == 3) {
    printf("loading API:  %s\n", argv[1]);
    handleCommands(argv[1], argv[2], ioRef);
  } else {
    printf("USAGE\n\n");
    printf("aRobotApp <apiname> dump\tDumps properties and objects.\n");
    printf("aRobotApp <apiname> demo\tRuns a demo for the chosen API.\n");
  }

  aIO_ReleaseLibRef(ioRef, NULL);
  aLeakCheckCleanup();

} // main //
