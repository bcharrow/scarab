/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aSimpleRange.c                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of the aSimpleRange application.    */
/*		The design, creation, and use of this code is      */
/*		documented at:                                     */
/*                                                                 */
/* www.acroname.com/brainstem/tutorials/datalogger/datalogger.html */
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

#include "aIO.h"
#include "aStem.h"
#include "aCmd.tea"
#include "aModuleVM.h"
#include "aModuleUtil.h"
#include "aGP2D02.h"

#include "aSimpleRange.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * the setting file defines
 */

#define MAXSETTINGLEN 		32

#define CONFIGFILENAME		"aSimpleRange.config"

#define BAUDRATEKEY		"baudrate"
#define DEFAULTBAUDRATE 	9600

#define MODULEKEY		"module"
#define DEFAULTMODULE		2

#define MEASUREMENTSKEY		"measurements"
#define DEFAULTNUMMEASURMENTS	100

#define PORTNAMEKEY		"portname"
#ifdef aWIN
#define DEFAULTPORTNAME 	"COM1"
#endif /* aWIN */
#ifdef aUNIX
#ifdef aMACX
#define DEFAULTPORTNAME 	"tty.USA28X111P1.1"
#else /* aMACX */
#define DEFAULTPORTNAME 	"ttyS0"
#endif /* aMACX */
#endif /* aUNIX */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * the state structure for the application 
 */

typedef struct aSimpleRange {
  aIOLib		ioRef;
  aStemLib		stemRef;
  aSettingFileRef	settingFile;
  unsigned char		module;
  int			baudRate;
  char			portName[MAXSETTINGLEN];
  aStreamRef		linkStream;
  aStreamRef		outputStream;
  int			nMeasurements;
} aSimpleRange;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */

static void show_int_param(aStreamRef out, char* desc, int param);
static void show_string_param(aStreamRef out, char* desc, char* param);
static aErr initialize(aSimpleRange* p02, aStreamRef output);
static aErr get_settings(aSimpleRange* p02);
static aErr build_link(aSimpleRange* p02);
static aErr measure(aSimpleRange* p02);
static aErr cleanup(aSimpleRange* p02);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * show_int_param
 *
 * Displays a formatted int parameter message on the given stream.
 */

void show_int_param(aStreamRef out, char* desc, int param)
{
  char line[100]; /* these could overflow if not careful */
  char num[10];

  /* format the line */
  aStringCopy(line, desc);
  aStringCat(line, ": ");
  aStringFromInt(num, param);
  aStringCat(line, num);

  /* dump it to the output stream */
  aStream_WriteLine(aStreamLibRef(out), out, line, NULL);

} /* show_int_param */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * show_string_param
 *
 * Displays a formatted string parameter message on the given stream.
 */
void show_string_param(aStreamRef out, char* desc, char* param)
{
  char line[100]; /* these could overflow if not careful */

  /* format the line */
  aStringCopy(line, desc);
  aStringCat(line, ": ");
  aStringCat(line, param);

  /* dump it to the output stream */
  aStream_WriteLine(aStreamLibRef(out), out, line, NULL);

} /* show_string_param */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * initialize
 *
 * Set up the state structure for the application and get access
 * to the aIO and aStem libraries for use later.
 */

aErr initialize(aSimpleRange* p02, aStreamRef output)
{
  aErr err = aErrNone;

  aBZero(p02, sizeof(aSimpleRange));

  if (err == aErrNone)
    p02->ioRef = aStreamLibRef(output);

  if (err == aErrNone)
    aStem_GetLibRef(&p02->stemRef, &err);

  if (err == aErrNone)
    p02->outputStream = output;
 
  return err;

} /* initialize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * get_settings
 *
 * Get the settings file object set up and read in the basic 
 * settings we need for the application.
 */

aErr get_settings(aSimpleRange* p02)
{
  aErr err = aErrNone;

  /* build a settings file */
  if (err == aErrNone)
    aSettingFile_Create(p02->ioRef, MAXSETTINGLEN, 
    	    		CONFIGFILENAME,
            		&p02->settingFile, &err);

  /* get the BrainStem module number */
  if (err == aErrNone) {
    int module;
    aSettingFile_GetInt(p02->ioRef, p02->settingFile,
    			MODULEKEY, 
    			&module, DEFAULTMODULE, &err);
    p02->module = (unsigned char)module;
  }

  /* get the number of measurements from the setting file */
  if (err == aErrNone)
    aSettingFile_GetInt(p02->ioRef, p02->settingFile,
    			MEASUREMENTSKEY, 
    			&p02->nMeasurements, 
    			DEFAULTNUMMEASURMENTS, &err);

  /* get the baud rate from the setting file */
  if (err == aErrNone)
    aSettingFile_GetInt(p02->ioRef, p02->settingFile,
    			BAUDRATEKEY, 
    			&p02->baudRate, 
    			DEFAULTBAUDRATE, &err);

  return err;

} /* get_settings */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * build_link
 *
 * Build the link to the BrainStem module, using the port name 
 * specified in the setting file.
 */

aErr build_link(aSimpleRange* p02)
{
  aErr err = aErrNone;
  char* pPortName;

  /* get the port name string and copy it into our state block */
  if (err == aErrNone) {
    aSettingFile_GetString(p02->ioRef, p02->settingFile,
            PORTNAMEKEY, &pPortName, DEFAULTPORTNAME, &err);
    aStringCopy(p02->portName, pPortName);
  }

  /* create the stream to the serial port */
  if (err == aErrNone) {
    aStream_CreateSerial(p02->ioRef, p02->portName, 
    			 (unsigned int)p02->baudRate,
    			 &p02->linkStream, &err);
    if (err != aErrNone) {
      aStream_WriteLine(p02->ioRef, p02->outputStream,
    		      "unable to open serial port", NULL);
      cleanup(p02);
    }
  }

  /* set this as the link port for the aStem library 
   * this link gets automatically destroyed when the aStem 
   * library is released */
  if (err == aErrNone)
    aStem_SetStream(p02->stemRef, p02->linkStream, 
    		    kStemModuleStream, &err);


  /* now, make sure the module is in place and bail if not */
  if ((err == aErrNone) &&
      !aModuleUtil_EnsureModule(p02->stemRef, p02->module)) {
    aStream_WriteLine(p02->ioRef, p02->outputStream,
    		      "active module not found", NULL);
    cleanup(p02);
    err = aErrIO;
  }

  return err;

} /* build_link */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * measure
 *
 * Take the measurements as fast as we can.
 */

aErr measure(aSimpleRange* p02)
{
  aErr err = aErrNone;
  int i, val;
  char num[10];

  /* do the loop to take the measurements */
  for (i = 0; (i < p02->nMeasurements)
  	      && (err == aErrNone); i++) {

    /* take the reading */
    err = aGP2D02_ReadInt(p02->stemRef, 
    			  p02->module,
    			  0, 
    			  &val);

    /* convert the number and display it */
    if (err == aErrNone) {
      aStringFromInt(num, val);
      aStream_WriteLine(p02->ioRef, p02->outputStream, num, &err);
    }      

  } /* for */

  return err;

} /* measure */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * cleanup
 *
 * Release all the resources acquired by this application to 
 * prepare for exit.
 */

aErr cleanup(aSimpleRange* p02)
{
  aErr err = aErrNone;

  if (p02->settingFile)
    aSettingFile_Destroy(p02->ioRef, p02->settingFile, &err);

  if (p02->stemRef)
    aStem_ReleaseLibRef(p02->stemRef, &err);

  return err;

} /* cleanup */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aSimpleRange_Execute
 *
 * The actual calling sequence for the routines.
 */

int aSimpleRange_Execute(aStreamRef output)
{
  aErr err = aErrNone;
  aSimpleRange  dl;

  /* output a console header */
  aStream_WriteLine(aStreamLibRef(output), output, "", &err);
  aStream_WriteLine(aStreamLibRef(output), output,
  		    "aSimpleRange Application", &err);
  aStream_WriteLine(aStreamLibRef(output), output, "", &err);

  if (err == aErrNone)
    err = initialize(&dl, output);

  if (err == aErrNone)
    err = get_settings(&dl);

  if (err == aErrNone)
    err = build_link(&dl);

  /* report what we are up to the output stream */
  if (err == aErrNone) {
    show_int_param(output, "module number", dl.module);
    show_int_param(output, "baud rate", dl.baudRate);
    show_string_param(output, "port name", dl.portName);
    show_int_param(output, "number of measurements", 
    		   dl.nMeasurements);
  }

  if (err == aErrNone)
    err = measure(&dl);

  if (err == aErrNone)
    err = cleanup(&dl);

  /* report the completion */
  aStream_WriteLine(aStreamLibRef(output), output,
  		    "", NULL);
  if (err == aErrNone)
    aStream_WriteLine(aStreamLibRef(output), output,
  		    "Measurements Complete", NULL);
  else
    aStream_WriteLine(aStreamLibRef(output), output,
  		    "Measurement Failed", NULL);
    
  return(err != aErrNone);

} /* aSimpleRange_Execute */
