/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: aDataLogger.c                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Implementation of the aDataLogger application.     */
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
#include "aModule.tea"
#include "aModuleVM.h"
#include "aModuleVal.h"
#include "aModuleUtil.h"
#include "aDataLogger.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * the setting file defines
 */
#define MAXSETTING 		32

#define CONFIGFILENAME		"aDataLogger.config"

#define BAUDRATEKEY		"baudrate"
#define DEFAULTBAUDRATE 	9600

#define DATAFILEKEY		"datafile"
#define DEFAULTDATAFILE		"aDataLogger.dat"

#define FREQUENCYKEY		"frequency"
#define DEFAULTFREQUENCY	5

#define MODULEKEY		"module"
#define DEFAULTMODULE		2

#define SLOTKEY			"slot"
#define DEFAULTSLOT		0

#define MEASUREMENTSKEY		"measurements"
#define DEFAULTNUMMEASURMENTS	100		

#define PORTNAMEKEY		"portname"
#ifdef aWIN
#define DEFAULTPORTNAME 	"COM1"
#endif /* aWIN */
#ifdef aUNIX
#define DEFAULTPORTNAME         "ttyS0"
#endif /* aUNIX */
#ifdef aWINCE
#define DEFAULTPORTNAME         "COM1:"
#endif /* aWINCE */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * the state structure for the application 
 */
typedef struct aDataLogger {
  aIOLib		ioRef;
  aStemLib		stemRef;
  aSettingFileRef	settingFile;
  unsigned char		module;
  unsigned char		slot;
  int			baudRate;
  char			portName[MAXSETTING];
  char			hbModeSave;
  aStreamRef		linkStream;
  aStreamRef		dataStream;
  aStreamRef		outputStream;
  int			nMeasurements;
  int			nFrequency;
  char			dataFileName[MAXSETTING];
} aDataLogger;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * local prototypes
 */
static void show_int_param(aStreamRef out, char* desc, int param);
static void show_string_param(aStreamRef out, char* desc, char* param);
static aErr initialize(aDataLogger* pDL, aStreamRef output);
static aErr get_settings(aDataLogger* pDL);
static aErr build_link(aDataLogger* pDL);
static aErr open_datafile(aDataLogger* pDL);
static aErr measure(aDataLogger* pDL);
static aErr cleanup(aDataLogger* pDL);


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

aErr initialize(aDataLogger* pDL, aStreamRef output)
{
  aErr err = aErrNone;

  aBZero(pDL, sizeof(aDataLogger));

  if (err == aErrNone)
    aIO_GetLibRef(&pDL->ioRef, &err);

  if (err == aErrNone)
    aStem_GetLibRef(&pDL->stemRef, &err);

  if (err == aErrNone)
    pDL->outputStream = output;
 
  return err;

} /* initialize */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * get_settings
 *
 * Get the settings file object set up and read in the basic 
 * settings we need for the application.
 */

aErr get_settings(aDataLogger* pDL)
{
  aErr err = aErrNone;

  if (err == aErrNone)
    aSettingFile_Create(pDL->ioRef, MAXSETTING, 
    	    		CONFIGFILENAME,
            		&pDL->settingFile, &err);

  /* get the BrainStem module number */
  if (err == aErrNone) {
    int module;
    aSettingFile_GetInt(pDL->ioRef, pDL->settingFile,
    			MODULEKEY, 
    			&module, DEFAULTMODULE, &err);
    pDL->module = (unsigned char)module;
  }

  /* get the file slot number */
  if (err == aErrNone) {
    int slot;
    aSettingFile_GetInt(pDL->ioRef, pDL->settingFile,
    			SLOTKEY, 
    			&slot, DEFAULTSLOT, &err);
    pDL->slot = (unsigned char)slot;
  }

  /* get the number of measurements from the setting file */
  if (err == aErrNone)
    aSettingFile_GetInt(pDL->ioRef, pDL->settingFile,
    			MEASUREMENTSKEY, 
    			&pDL->nMeasurements, 
    			DEFAULTNUMMEASURMENTS, &err);

  /* get the frequency of measurement from the setting file */
  if (err == aErrNone)
    aSettingFile_GetInt(pDL->ioRef, pDL->settingFile,
    			FREQUENCYKEY, 
    			&pDL->nFrequency, 
    			DEFAULTFREQUENCY, &err);

  /* get the baud rate from teh setting file */
  if (err == aErrNone)
    aSettingFile_GetInt(pDL->ioRef, pDL->settingFile,
    			BAUDRATEKEY, 
    			&pDL->baudRate, 
    			DEFAULTBAUDRATE, &err);

  return err;

} /* get_settings */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * build_link
 *
 * Build the link to the BrainStem module, using the port name 
 * specified in the setting file.
 */

aErr build_link(aDataLogger* pDL)
{
  aErr err = aErrNone;
  char* pPortName;

  /* get the port name string and copy it into our state block */
  if (err == aErrNone) {
    aSettingFile_GetString(pDL->ioRef, pDL->settingFile,
            PORTNAMEKEY, &pPortName, DEFAULTPORTNAME, &err);
    aStringCopy(pDL->portName, pPortName);
  }

  /* create the stream to the serial port */
  if (err == aErrNone) {
    aStream_CreateSerial(pDL->ioRef, pDL->portName, 
    			 (unsigned int)pDL->baudRate,
    			 &pDL->linkStream, &err);
    if (err != aErrNone) {
      aStream_WriteLine(pDL->ioRef, pDL->outputStream,
    		      "unable to open serial port", NULL);
      cleanup(pDL);
    }
  }

  /* set this as the link port for the aStem library 
   * this link gets automatically destroyed when the aStem 
   * library is released */
  if (err == aErrNone)
    aStem_SetStream(pDL->stemRef, pDL->linkStream, 
    		    kStemModuleStream, &err);


  /* now, make sure the module is in place and bail if not */
  if ((err == aErrNone) &&
      !aModuleUtil_EnsureModule(pDL->stemRef, pDL->module)) {
    aStream_WriteLine(pDL->ioRef, pDL->outputStream,
    		      "active module not found", NULL);
    cleanup(pDL);
    err = aErrIO;
  }
  
  /* finally, store the old heartbeat mode and set auto heartbeat */
  if (err == aErrNone) {
    err = aModuleVal_Get(pDL->stemRef, pDL->module, 
			 aMODULE_VAL_HBFLAG, 
    			 &pDL->hbModeSave);
    if ((err == aErrNone) && (pDL->hbModeSave != 1))
      err = aModuleVal_Set(pDL->stemRef, pDL->module, 
			   aMODULE_VAL_HBFLAG, 1);
  }

  return err;

} /* build_link */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * open_datafile
 *
 * Open the file that will store the results of the measurements.
 */

aErr open_datafile(aDataLogger* pDL)
{
  aErr err = aErrNone;
  char* pFileName;

  if (err == aErrNone)
    aSettingFile_GetString(pDL->ioRef, pDL->settingFile,
            DATAFILEKEY, &pFileName, DEFAULTDATAFILE, &err);

  if (err == aErrNone) {
    aStringCopy(pDL->dataFileName, pFileName);
    aStream_CreateFileOutput(pDL->ioRef, pDL->dataFileName, 
    			     aFileAreaUser, &pDL->dataStream,
    			     &err);
  }

  return err;

} /* open_datafile */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * measure
 *
 * Take the measurements from the BrainStem module's TEA program.
 */

aErr measure(aDataLogger* pDL)
{
  aErr err = aErrNone;
  int i, val;
  unsigned long then, now, ticks;
  char num[10];

  /* compute the number of ticks that will give us the correct
   * frequency */
  ticks = (unsigned long)1000 / pDL->nFrequency;
  
  /* find out the time right now */
  aIO_GetMSTicks(pDL->ioRef, &then, &err);

  /* do the loop to take the measurements */
  for (i = 0; (i < pDL->nMeasurements)
  	      && (err == aErrNone); i++) {
    err = aModuleVM_Execute(pDL->stemRef, pDL->module,
    			    0, 0, NULL, &val);
    
    /* convert the number and store it */
    if (err == aErrNone) {
      aStringFromInt(num, val);
      aStream_WriteLine(pDL->ioRef, pDL->dataStream, num, &err);
    }

    /* stall for the remaining time before the next measurement */
    if (err == aErrNone)
      aIO_GetMSTicks(pDL->ioRef, &now, &err);
      
    /* if the time is too quick, we may oversleep so bail */
    if ((ticks - (now - then)) < 6) {
      aStream_WriteLine(pDL->ioRef, pDL->outputStream,
    		      "Error, frequency too fast, try a faster baudrate.", NULL);
      cleanup(pDL);
      err = aErrTimeout;
    }
    if (err == aErrNone)
      aIO_MSSleep(pDL->ioRef, ticks - (now - then), &err);
    if (err == aErrNone)
      aIO_GetMSTicks(pDL->ioRef, &then, &err);
  }

  return err;

} /* measure */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * cleanup
 *
 * Release all the resources acquired by this application to 
 * prepare for exit.
 */

aErr cleanup(aDataLogger* pDL)
{
  aErr err = aErrNone;

  /* restore the previous heartbeat mode */
  if (pDL->linkStream)
    aModuleVal_Set(pDL->stemRef, pDL->module, 
    		   aMODULE_VAL_HBFLAG, pDL->hbModeSave);

  if (pDL->dataStream)
    aStream_Destroy(pDL->ioRef, pDL->dataStream, &err);

  if (pDL->settingFile)
    aSettingFile_Destroy(pDL->ioRef, pDL->settingFile, &err);

  if (pDL->ioRef)
    aIO_ReleaseLibRef(pDL->ioRef, &err);

  if (pDL->stemRef)
    aStem_ReleaseLibRef(pDL->stemRef, &err);

  return err;

} /* cleanup */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * aDataLogger_Execute
 *
 * The actual calling sequence for the routines.
 */

int aDataLogger_Execute(aStreamRef output)
{
  aErr err = aErrNone;
  aDataLogger  dl;

  /* output a console header */
  aStream_WriteLine(aStreamLibRef(output), output, "", &err);
  aStream_WriteLine(aStreamLibRef(output), output,
  		    "aDataLogger Application", &err);
  aStream_WriteLine(aStreamLibRef(output), output, "", &err);

  if (err == aErrNone)
    err = initialize(&dl, output);

  if (err == aErrNone)
    err = get_settings(&dl);

  if (err == aErrNone)
    err = open_datafile(&dl);

  if (err == aErrNone)
    err = build_link(&dl);

  /* report what we are up to the output stream */
  if (err == aErrNone) {
    show_int_param(output, "module number", dl.module);
    show_int_param(output, "TEA file slot number", dl.slot);
    show_int_param(output, "baud rate", dl.baudRate);
    show_string_param(output, "port name", dl.portName);
    show_int_param(output, "number of measurements", 
    		   dl.nMeasurements);
    show_int_param(output, "measurement frequency (Hz)", 
    		   dl.nFrequency);
    show_string_param(output, "data file name (in aUser directory)", dl.dataFileName);
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

} /* aDataLogger_Execute */
