/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotPackager.h 	  		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of Robot API data package routines.     //
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

#ifndef _acpRobotPackager_H_
#define _acpRobotPackager_H_

#include "aIO.h"
#include "aSteep.h"

#include "acpList.h"
#include "acpString.h"
#include "acpRobotPackage.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

class acpRobotPackager {
  public:
    				acpRobotPackager(
    				  aStreamRef logStream = NULL);
    				~acpRobotPackager();

    int				handleCommands(
    				  const int nArgs,
    				  const char* pArgs[]);
    
    void			log(
    				  const char* pMsg);
    void			error(
    				  const char* pMsg);

    aStreamRef			getLogStream()
				  { return m_logStream; }
    void			updateSteepFailures(
				  const int n)
				  { m_nSteepFailures += n; }
    void			updateImportErrors(
				  const int n)
				  { m_nImportErrors += n; }

    // XML import handling routines
    acpRobotPackage*		importPackage(
    				  const char* pFileName,
    				  const aFileArea eFileArea);
    acpPackage*			readPackage(
    				  const char* pFileName);

    void			compileFiles(
    				  acpRobotPackage* pPkg);
    void			checkTemplateReferences(
    				  acpRobotPackage* pPkg);
    void			checkLinkReferences(
    				  acpRobotPackage* pPkg);


  private:

    void			loadCUP(
				  const char* pName,
				  unsigned long* pSize,
				  char** pBuff,
				  aErr* pErr);

    void			compileTEA(
				  const char* pName,
				  char** ppCup,
				  unsigned long* ulCupSize,
				  aErr* pErr);

  private:
    aIOLib			m_ioRef;
    aSteepLib			m_steepRef;
    aStreamRef			m_logStream;
    
    // command input processing
    int				m_nInputErrors;
    int				m_nImportErrors;
    int				m_nSteepFailures;
    acpList<acpString>		m_inputFileNames;
  
  friend class acpPkgImport;
};

class acpRobotPkgImport :
  public acpPkgImport
{
  public:

				acpRobotPkgImport(
				  aIOLib ioRef,
				  aStreamRef logStream,
				  aStreamRef input,
				  acpPackage* pPackage
				) :
				  acpPkgImport(
				    ioRef,
				    logStream,
				    input,
				    pPackage)
				{}

    virtual			~acpRobotPkgImport()
				  {}

    virtual void		fillInTags();

    void			addTag(
				  acpPkgXML* pXMLOrigin,
				  acpPackageTag* pTag);

};

#endif // _acpRobotPackager_H_
