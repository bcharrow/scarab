/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: stdio_aGarciaApp.c					   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of a stdio-based Garcia             */
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

#include "acpGarcia.h"


/////////////////////////////////////////////////////////////////////

class acpGarciaAppExecute :
  public acpCallback
{
  public:
	  		acpGarciaAppExecute(
  			  acpGarcia* pcGarcia,
  			  acpObject* pBehavior
  			) :
  			  m_pcGarcia(pcGarcia),
  			  m_pBehavior(pBehavior) 
  			{}

    aErr		call();

  private:
    acpGarcia*	 	m_pcGarcia;
    acpObject*		m_pBehavior;
};


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr acpGarciaAppExecute::call()
{
  printf("behavior \"%s\" with id %d executing...\n", 
  	 m_pBehavior->getNamedValue("name")->getStringVal(),
  	 m_pBehavior->getNamedValue("unique-id")->getIntVal());

  return aErrNone;

} /* sExecute callback*/



/////////////////////////////////////////////////////////////////////

class acpGarciaAppComplete :
  public acpCallback
{
  public:
	  		acpGarciaAppComplete(
  			  acpGarcia* pcGarcia,
  			  acpObject* pBehavior
  			) :
  			  m_pcGarcia(pcGarcia),
  			  m_pBehavior(pBehavior) 
  			{}

    aErr		call();
    
  private:
    acpGarcia*	 	m_pcGarcia;
    acpObject*		m_pBehavior;
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

aErr acpGarciaAppComplete::call()
{
  printf("completed with status = %d\n", 
  	 m_pBehavior->getNamedValue("completion-status")->getIntVal());

  return aErrNone;

} /* sComplete callback*/



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * main
 */

int main(
  int argc, 
  char* argv[])
{
  aErr err;
  aIOLib ioRef;
  acpGarcia garcia;
  acpObject* pBehavior;

  aIO_GetLibRef(&ioRef, &err);

  printf("waiting for garcia\n");

  while (!garcia.getNamedValue("active")->getBoolVal()) {
    printf("still waiting\n");
    aIO_MSSleep(ioRef, 100, NULL);
  }

  printf("queueing a simple triangle\n");

  acpValue executeCB;
  acpValue completeCB;
  acpValue lengthVal(0.5f);
  acpValue rotationVal((float)(3.1459 * 2 / 3));
  pBehavior = garcia.createNamedBehavior("move", "out");
  pBehavior->setNamedValue("distance", &lengthVal);
  executeCB.set(new acpGarciaAppExecute(&garcia, pBehavior));
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(&garcia, pBehavior));
  pBehavior->setNamedValue("completion-callback", &completeCB);
  garcia.queueBehavior(pBehavior);

  pBehavior = garcia.createNamedBehavior("pivot", "pivot 1");
  pBehavior->setNamedValue("angle", &rotationVal);
  executeCB.set(new acpGarciaAppExecute(&garcia, pBehavior));
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(&garcia, pBehavior));
  pBehavior->setNamedValue("completion-callback", &completeCB);
  garcia.queueBehavior(pBehavior);

  pBehavior = garcia.createNamedBehavior("move", "over");
  pBehavior->setNamedValue("distance", &lengthVal);
  executeCB.set(new acpGarciaAppExecute(&garcia, pBehavior));
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(&garcia, pBehavior));
  pBehavior->setNamedValue("completion-callback", &completeCB);
  garcia.queueBehavior(pBehavior);

  // we don't set the callbacks on this behavior so there will
  // be no display information about this pivot behavior
  pBehavior = garcia.createNamedBehavior("pivot", "pivot 2");
  pBehavior->setNamedValue("angle", &rotationVal);
  garcia.queueBehavior(pBehavior);

  pBehavior = garcia.createNamedBehavior("move", "back");
  pBehavior->setNamedValue("distance", &lengthVal);
  executeCB.set(new acpGarciaAppExecute(&garcia, pBehavior));
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(&garcia, pBehavior));
  pBehavior->setNamedValue("completion-callback", &completeCB);
  garcia.queueBehavior(pBehavior);

  pBehavior = garcia.createNamedBehavior("pivot", "pivot 3");
  pBehavior->setNamedValue("angle", &rotationVal);
  executeCB.set(new acpGarciaAppExecute(&garcia, pBehavior));
  pBehavior->setNamedValue("execute-callback", &executeCB); 
  completeCB.set(new acpGarciaAppComplete(&garcia, pBehavior));
  pBehavior->setNamedValue("completion-callback", &completeCB);
  garcia.queueBehavior(pBehavior);

  // this loop checks to see when the list above is complete
  // and checks for callbacks in 100 mSec intervals  
  while (!garcia.getNamedValue("idle")->getBoolVal())
    garcia.handleCallbacks(100);

  printf("done\n");

  aIO_ReleaseLibRef(ioRef, &err);

  return 0;

} /* main */
