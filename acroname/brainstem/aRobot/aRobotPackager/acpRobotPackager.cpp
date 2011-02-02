/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotPackager.cpp 	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of robot API data package           //
//		routines.					   //
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


#if defined(aMACX) || defined(aUNIX)
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#endif


#include "aStream_STDIO_Console.h"
#include "aVersion.h"
#include "aUtil.h"
#include "aSteepText.h"

#include "acpPkgImport.h"
#include "acpException.h"
#include "acpRobotPackage.h"
#include "acpRobotPackager.h"

#include "acpXmlTag_apiPACKAGE.h"
#include "acpXmlTag_apiNAME.h"
#include "acpXmlTag_PROPERTY.h"
#include "acpXmlTag_PRIMITIVE.h"
#include "acpXmlTag_TEMPLATE.h"
#include "acpXmlTag_USEROBJECT.h"
#include "acpXmlTag_TYPE.h"
#include "acpXmlTag_DEFAULT.h"
#include "acpXmlTag_APIFILE.h"
#include "acpXmlTag_STEMFILE.h"
#include "acpXmlTag_SET.h"
#include "acpXmlTag_GET.h"
#include "acpXmlTag_CONVFAC.h"
#include "acpXmlTag_PARAMSIZE.h"
#include "acpXmlTag_BUFFERSIZE.h"
#include "acpXmlTag_TEMPLATENAME.h"
#include "acpXmlTag_INDEX.h"
#include "acpXmlTag_INITIALIZER.h"
#include "acpXmlTag_DISTANCEUNIT.h"
#include "acpXmlTag_ANGLEUNIT.h"
#include "acpXmlTag_MASSUNIT.h"
#include "acpXmlTag_DESCRIPTION.h"
#include "acpXmlTag_LINK.h"
#include "acpXmlTag_LINKNAME.h"

#include "acpTag_apiPACKAGE.h"
#include "acpTag_PROPERTY.h"
#include "acpTag_PRIMITIVE.h"
#include "acpTag_TEMPLATE.h"
#include "acpTag_USEROBJECT.h"
#include "acpTag_SET.h"
#include "acpTag_GET.h"
#include "acpTag_INITIALIZER.h"
#include "acpTag_LINK.h"



int findString(acpList<acpString>& rlist, acpString& rstr);
void getFilePath(char* path, const char* filename);
bool osEnsureDirectory(const char* pFullpath);
void getFileName(char* filename, const char* fullpath);

int findString(acpList<acpString>& rlist, acpString& rstr)
{
  acpListIterator<acpString> iter(rlist);
  acpString* pName;
  int nFoundCt = 0;
  while ((pName = iter.next())) {
    if (!aStringCompare((char*)(*pName), (char*)rstr)) {
      nFoundCt++;
    }
  }
  return nFoundCt;
}

void getFilePath(char* path, const char* filename)
{
  unsigned int len = (unsigned int)aStringLen(filename);
  char* p = (char*)&filename[len-1];
  
  while (p != filename) {
    if (*p == '/')
      break;
    p--;
    len--;
  }

  if (len > 1) {
    len--;
    aStringNCopy(path, filename, len);
    path[len] = 0;
  } else {
    aStringCopy(path, "");
  }

}

void getFileName(char* filename, const char* fullpath)
{
  unsigned int len = (unsigned int)aStringLen(fullpath);
  char* p = (char*)&fullpath[len-1];
  
  while (p != fullpath) {
    if (*p == '/')
      break;
    p--;
  }

  if (*p == '/')
    p++;
  aStringCopy(filename, p);
}



// directory checker
// this should be the only OS specific code in RobotPackager
#if defined(aMACX) || defined(aUNIX)
bool osEnsureDirectory(
  const char* pFullPath
)
{
  if (mkdir(pFullPath, (S_IRWXU|S_IRWXG|S_IRWXO))) {
    int e = errno;
    switch (e) {
    case  EEXIST:
      return true;
      break;
    default:
      return false;
    }
  }
  return true;
}
#endif

#ifdef aWIN
bool osEnsureDirectory(
  const char* pFullpath)
{
  CreateDirectory(pFullpath, NULL);
  return true;
}
#endif




/////////////////////////////////////////////////////////////////////

void acpRobotPkgImport::fillInTags()
{
  addRecognizedTag(new apiPACKAGE(this));
  addRecognizedTag(new apiNAME(this));
  addRecognizedTag(new PROPERTY(this));
  addRecognizedTag(new PRIMITIVE(this));
  addRecognizedTag(new TEMPLATE(this));
  addRecognizedTag(new USEROBJECT(this));
  addRecognizedTag(new TYPE(this));
  addRecognizedTag(new DEFAULT(this));
  addRecognizedTag(new APIFILE(this));
  addRecognizedTag(new STEMFILE(this));
  addRecognizedTag(new SET(this));
  addRecognizedTag(new GET(this));
  addRecognizedTag(new CONVFAC(this));
  addRecognizedTag(new PARAMSIZE(this));
  addRecognizedTag(new BUFFERSIZE(this));
  addRecognizedTag(new TEMPLATENAME(this));
  addRecognizedTag(new INDEX(this));
  addRecognizedTag(new INITIALIZER(this));
  addRecognizedTag(new DISTANCEUNIT(this));
  addRecognizedTag(new ANGLEUNIT(this));
  addRecognizedTag(new MASSUNIT(this));
  addRecognizedTag(new DESCRIPTION(this));
  addRecognizedTag(new LINK(this));
  addRecognizedTag(new LINKNAME(this));
}


/////////////////////////////////////////////////////////////////////

void acpRobotPkgImport::addTag(
  acpPkgXML* pXMLOrigin,
  acpPackageTag* pTag) 
{
  acpPkgImport::addTag(pXMLOrigin, pTag);
  m_pPackage->addTag(pTag);
}


/////////////////////////////////////////////////////////////////////

acpRobotPackager::acpRobotPackager(
  aStreamRef logStream // = NULL
) :
  m_nInputErrors(0),
  m_nImportErrors(0),
  m_nSteepFailures(0)
{
  aErr err;

  if (aIO_GetLibRef(&m_ioRef, &err))
    throw acpException(err, "unable to open IO library");

  if (logStream) {
    m_logStream = logStream;
  } else {
    err = aStream_Create_STDIO_Console_Output(m_ioRef, &m_logStream);
    if (err != aErrNone)
      throw acpException(err, "unable to create log stream");
  }

  log("Acroname Robot API Packager");
  acpString line("Version ");
  line += aPACKAGE_MAJOR;
  line += '.';
  line += aPACKAGE_MINOR;
  line += ", build ";
  line += aPACKAGE_BUILD_NUM;
  log(line);
  log(aCOPYRIGHT_LINE);
}


/////////////////////////////////////////////////////////////////////

acpRobotPackager::~acpRobotPackager()
{
  if (m_logStream) {
    aStream_Destroy(m_ioRef, m_logStream, NULL);
    m_logStream = NULL;
  }

  if (m_ioRef) {
    aIO_ReleaseLibRef(m_ioRef, NULL);
    m_ioRef = NULL;
  }
  
}


/////////////////////////////////////////////////////////////////////
    
int acpRobotPackager::handleCommands(
  const int nArgs,
  const char* pArgs[])
{
  int i;
  
  aAssert(nArgs > 0);

  for (i = 1; i < nArgs; i++) {
    switch (*pArgs[i]) {
    
    case '-':
      break;

    default:
      {
        unsigned int len = aStringLen(pArgs[i]);
        if (len > aFILE_NAMEMAXCHARS) {
          acpString msg("filename too long: ");
          msg += pArgs[i];
          error(msg);
          m_nInputErrors++;
        } else {
          m_inputFileNames.add(new acpString(pArgs[i]));
        }
      }
      break;

    } // switch
  } // for

  if (m_nInputErrors) {
    acpString msg(m_nInputErrors);
    msg += " command line errors";
    error(msg);
  } else {
    acpListIterator<acpString> fileIterator(m_inputFileNames);
    acpString* pFileName;
    while ((pFileName = fileIterator.next())) {

      log ("importing xml");

      try {

        bool bGo = true;
        char sExt[40];
        acpString sFileName(*pFileName);

        // apply default extension if extension not given
        aUtil_GetFileExtension(sExt, sFileName, NULL);
        if (!aStringCompare("", sExt)) {
          sFileName += ".xml";
        }

        // first we import the XML and check its form
        acpRobotPackage* pPackage = importPackage(sFileName, 
      					     aFileAreaUser);

        // if import okay then we check link references
        if (bGo && m_nImportErrors == 0) {
          log("*** checking link stream references");
          checkLinkReferences(pPackage);
        } else if (bGo) {
          acpString msg("Encountered ");
          msg += (long)m_nImportErrors;
          msg += " import error(s).";
          log(msg);
          bGo = false;
        }

        // if import okay then we check template references
        if (bGo && m_nImportErrors == 0) {
          log("*** checking template references");
          checkTemplateReferences(pPackage);
        } else if (bGo) {
          acpString msg("Encountered ");
          msg += (long)m_nImportErrors;
          msg += " import error(s).";
          log(msg);
          bGo = false;
        }

        // if references okay then we compile the referenced files
        if (bGo && m_nImportErrors == 0) {
          log("*** steeping referenced TEA files ");
          compileFiles(pPackage);
        } else if (bGo) {
          acpString msg("Encountered ");
          msg += (long)m_nImportErrors;
          msg += " import error(s).";
          log(msg);
          bGo = false;
        }

        // if compile okay then we write the package
        if (bGo && m_nSteepFailures == 0) {
          log("*** generating package");

          // write out with settings
          char resultFileRoot[aFILE_NAMEMAXCHARS];
          char resultFile[aFILE_NAMEMAXCHARS];
          aString_GetFileRoot(resultFileRoot, *pFileName);
          getFileName(resultFile, resultFileRoot);
          aStringCat(resultFile, aROBOT_PKG_EXTENSION);
          aErr err;
          aStreamRef resultStream = NULL;

          aStream_CreateFileOutput(m_ioRef, 
				   resultFile,
				   aFileAreaObject,
				   &resultStream,
				   &err);

          if (err != aErrNone) {
            if (err == aErrIO) {
              log("file IO or access error");
              log("no package generated");
            } else {
              throw acpException(err, "creating package output file");
            }
          }

          if (err == aErrNone) {
            acpString msg("writing ");
            msg += (long)pPackage->numTags();
            msg += " package tags";
            log(msg);
            pPackage->writeToStream(resultStream);
          }        

          if (resultStream != NULL) {
            if (aStream_Destroy(m_ioRef, resultStream, &err))
              throw acpException(err, "destroying output file");
          }

        } else if (bGo) {
          acpString msg("Failed to compile ");
          msg += (long)m_nSteepFailures;
          msg += " file(s).";
          log(msg);
          bGo = false;
        }

        if (!bGo)
          log("Packaging has halted!!!");

        delete pPackage;

      } catch (acpException& e) {
      	m_nImportErrors++;
        error(e);
      }

    } // while
    
  }

  return m_nInputErrors + m_nImportErrors;
}


/////////////////////////////////////////////////////////////////////
    
void acpRobotPackager::log(
  const char* pMsg)
{
  aStream_WriteLine(m_ioRef, m_logStream, pMsg, NULL);
}


/////////////////////////////////////////////////////////////////////
    
void acpRobotPackager::error(
  const char* pMsg)
{
  aStream_WriteLine(m_ioRef, m_logStream, pMsg, NULL);
}


/////////////////////////////////////////////////////////////////////

acpRobotPackage* acpRobotPackager::importPackage(
  const char* pFileName,
  const aFileArea eFileArea
)
{
  aErr err;
  aStreamRef input;

  // read the xml in
  if (aStream_CreateFileInput(m_ioRef, pFileName, eFileArea, 
  			      &input, &err))
  {
    log("Check file name or path");
    throw acpException(err, "opening xml import file");
  }

  acpRobotPackage* pPackage = new acpRobotPackage(m_ioRef);
  pPackage->initialize();

  acpRobotPkgImport importer(m_ioRef, m_logStream, input, pPackage);
  importer.fillInTags();
  importer.import(input);
  m_nImportErrors = importer.numErrors();

  return pPackage;
}


/////////////////////////////////////////////////////////////////////

acpPackage* acpRobotPackager::readPackage(
  const char* pFileName
)
{
  aErr err = aErrNone;
  aStreamRef packageStream;
  
  if (aStream_CreateFileInput(m_ioRef, 
  			      pFileName, 
  			      aFileAreaObject,
  			      &packageStream, 
  			      &err))
    throw acpException(err, "unable to open package");

  acpPackage* pPackage = new acpRobotPackage(m_ioRef, packageStream);

  if (aStream_Destroy(m_ioRef, packageStream, &err))
    throw acpException(err, "unable to close package file stream");

  return pPackage;
}


/////////////////////////////////////////////////////////////////////

void acpRobotPackager::compileFiles(
  acpRobotPackage* pPkg
)
{
  aErr err = aErrNone;

  if (aSteep_GetLibRef(&m_steepRef, &err))
    throw acpException(err, "Steep Library error");

  acpPackageTag* pTag;
  acpListIterator<acpPackageTag> tags(pPkg->m_tags);
  while ((pTag = tags.next())) {
    switch (pTag->getTagType()) {
      case aTAG_PRIMITIVE:
      {
        acpTag_PRIMITIVE* pPrim = (acpTag_PRIMITIVE*)pTag;
        if (pPrim->m_apifile.length()) {
          log(pPrim->m_apifile);
          compileTEA(
            pPrim->m_apifile,
            &pPrim->m_pApiCup,
            &pPrim->m_ulApiCupSize,
            &err);
          if (err != aErrNone)
            updateSteepFailures(1);
        }
        if (pPrim->m_stemfile.length()) {
          log(pPrim->m_stemfile);
          compileTEA(
            pPrim->m_stemfile,
            &pPrim->m_pStemCup,
            &pPrim->m_ulStemCupSize,
            &err);
          if (err != aErrNone)
            updateSteepFailures(1);
        }
        break;
      }
      case aTAG_SET:
      {
        acpTag_SET* pSet = (acpTag_SET*)pTag;
        if (pSet->m_apifile.length()) {
          log(pSet->m_apifile);
          compileTEA(
            pSet->m_apifile,
            &pSet->m_pApiCup,
            &pSet->m_ulApiCupSize,
            &err);
          if (pSet->m_pApiCup) {
            if (pSet->m_pApiCup[4]) {
              // non-void return value is illegal in property routine
              log("a PROPERTY SET routine must have a void return type");
              err = aErrParam;
            }
            int nParams = pSet->m_pApiCup[5];
            if ((nParams != 1) && (nParams != 2) && (nParams != 4)) {
              // input arguments in a get routine is illegal
              log("a PROPERTY SET routine has 1, 2, or 4 input bytes");
              err = aErrParam;
            }
          }
          if (err != aErrNone)
            updateSteepFailures(1);
        }
        break;
      }
      case aTAG_GET:
      {
        acpTag_GET* pGet = (acpTag_GET*)pTag;
        if (pGet->m_apifile.length()) {
          log(pGet->m_apifile);
          compileTEA(
            pGet->m_apifile,
            &pGet->m_pApiCup,
            &pGet->m_ulApiCupSize,
            &err);
          if (pGet->m_pApiCup) {
            if (pGet->m_pApiCup[5]) {
              // input arguments in a get routine is illegal
              log("a PROPERTY GET routine can not have input parameters");
              err = aErrParam;
            }
            if (pGet->m_pApiCup[4]) {
              // non-void return value is illegal in property routine
              log("a PROPERTY GET routine must have a void return type");
              err = aErrParam;
            }
          }
          if (err != aErrNone)
            updateSteepFailures(1);
        }
        break;
      }
    }
  }

  aSteep_ReleaseLibRef(m_steepRef, NULL);
}


/////////////////////////////////////////////////////////////////////

void acpRobotPackager::checkTemplateReferences(
  acpRobotPackage* pPkg
)
{
  acpList<acpString> listTemplates;
  acpList<acpString> listNamedProps;
  acpList<acpString> listNamedObjects;
  acpList<acpString> listDuplicates;

  acpPackageTag* pParentTag;
  acpPackageTag* pTag;

  // collect object names (templates, primitives, user objects)
  // collect template names
  // collect parent+property names
  // collect any duplicated names
  acpListIterator<acpPackageTag> tags1(pPkg->m_tags);
  while ((pTag = tags1.next())) {
    int nFound = 0;
    acpString dupTag;
    acpString* pName = NULL;
    switch (pTag->getTagType()) {
      case aTAG_PRIMITIVE:
      {
        dupTag = "PRIMITIVE";
        acpTag_PRIMITIVE* pPrimTag = (acpTag_PRIMITIVE*)pTag;
        pName = new acpString(pPrimTag->m_name);
        nFound = findString(listNamedObjects, pPrimTag->m_name);
        if (!nFound) listNamedObjects.add(pName);
        break;
      }
      case aTAG_USEROBJECT:
      {
        dupTag = "USEROBJECT";
        acpTag_USEROBJECT* pObjTag = (acpTag_USEROBJECT*)pTag;
        pName = new acpString(pObjTag->m_name);
        nFound = findString(listNamedObjects, pObjTag->m_name);
        if (!nFound) listNamedObjects.add(pName);
        break;
      }
      case aTAG_TEMPLATE:
      {
        dupTag = "TEMPLATE";
        acpTag_TEMPLATE* pTemplateTag = (acpTag_TEMPLATE*)pTag;
        pName = new acpString(pTemplateTag->m_name);
        nFound = findString(listNamedObjects, pTemplateTag->m_name);
        if (!nFound) 
          listNamedObjects.add(pName);
        if (!nFound) 
          listTemplates.add(new acpString(pTemplateTag->m_name));
        break;
      }
      case aTAG_PROPERTY:
      {
        dupTag = "PROPERTY";
        acpTag_PROPERTY* pPropTag = (acpTag_PROPERTY*)pTag;
        int kParent = pPropTag->getOwnerID();
        if (kParent) {
          acpString s("");
          pParentTag = pPkg->m_tags[kParent - 1];
          if (pParentTag->getTagType() == aTAG_PRIMITIVE) {
            acpTag_PRIMITIVE* p = (acpTag_PRIMITIVE*)pParentTag;
            s += p->m_name;
          }
          if (pParentTag->getTagType() == aTAG_TEMPLATE) {
            acpTag_TEMPLATE* p = (acpTag_TEMPLATE*)pParentTag;
            s += p->m_name;
          }
          s += "+";
          s += pPropTag->m_name;
          pName = new acpString(s);
        } else {
          pName = new acpString(pPropTag->m_name);
        }
        nFound = findString(listNamedProps, *pName);
        if (!nFound) 
          listNamedProps.add(pName);
        break;
      }
    } // switch

    if (nFound) {
      acpString msg;
      msg += "Duplicated identifier for a ";
      msg += dupTag;
      msg += ": ";
      msg += *pName;
      log(msg);
      listDuplicates.add(pName);
    }
  }

  // notify user of any duplicated identifiers
  if (listDuplicates.length()) {
    log("*** DUPLICATED IDENTIFIERS FOUND");
    updateImportErrors(listDuplicates.length());
    /*
    acpListIterator<acpString> iter(listDuplicates);
    acpString* pName;
    while (pName = iter.next()) {
      log(*pName);
    }
    */
  }

  // check each subobject template reference
  acpListIterator<acpPackageTag> tags2(pPkg->m_tags);
  while ((pTag = tags2.next())) {
    switch (pTag->getTagType()) {
      case aTAG_USEROBJECT:
      {
        acpTag_USEROBJECT* pObjTag = (acpTag_USEROBJECT*)pTag;
        int nFound = findString(listTemplates, pObjTag->m_templatename);
        if (!nFound) {
          acpString msg("In USEROBJECT ");
          msg += pObjTag->m_name;
          msg += ", TEMPLATENAME not recognized:  ";
          msg += pObjTag->m_templatename;
          log(msg);
          updateImportErrors(1);
        }
      }
    }
  }

  // check each subobject initializer reference
  acpListIterator<acpPackageTag> tags3(pPkg->m_tags);
  while ((pTag = tags3.next())) {
    switch (pTag->getTagType()) {
      case aTAG_INITIALIZER:
      {
        acpTag_INITIALIZER* pInitTag = (acpTag_INITIALIZER*)pTag;
        aShort ownerID = pTag->getOwnerID();
        if (ownerID) {
          pParentTag = pPkg->m_tags[ownerID - 1];
          aAssert(pParentTag->getTagType() == aTAG_USEROBJECT);
          acpTag_USEROBJECT* pObjTag = (acpTag_USEROBJECT*)pParentTag;

          acpString s(pObjTag->m_templatename);
          s += "+";
          s += pInitTag->m_name;

          int nFound = findString(listNamedProps, s);
          if (!nFound) {
            acpString msg("In USEROBJECT ");
            msg += pObjTag->m_name;
            msg += ", INITIALIZER name not recognized:  ";
            msg += pInitTag->m_name;
            log(msg);
            updateImportErrors(1);
          }
        }
      }
    }
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotPackager::checkLinkReferences(
  acpRobotPackage* pPkg
)
{
  acpList<acpString> listLinkNames;
  acpList<acpString> listDuplicates;

  acpPackageTag* pTag;

  // collect names of declared links
  acpListIterator<acpPackageTag> tags1(pPkg->m_tags);
  while ((pTag = tags1.next())) {
    int nFound = 0;
    acpString* pName = NULL;
    switch (pTag->getTagType()) {
      case aTAG_LINK:
      {
        acpTag_LINK* pLinkTag = (acpTag_LINK*)pTag;
        pName = new acpString(pLinkTag->m_name);
        nFound = findString(listLinkNames, pLinkTag->m_name);
        if (!nFound) listLinkNames.add(pName);
        break;
      }
    } // switch

    if (nFound) {
      listDuplicates.add(pName);
    }
  }

  // notify user of any duplicated link identifiers
  if (listDuplicates.length()) {
    log("*** DUPLICATE LINK DEFINITIONS FOUND");
    updateImportErrors(listDuplicates.length());
    acpListIterator<acpString> iter(listDuplicates);
    acpString* pName;
    while ((pName = iter.next())) {
      log(*pName);
    }
  } else {
    // echo declared links
    log("(default)");
    acpListIterator<acpString> iter(listLinkNames);
    acpString* pName;
    while ((pName = iter.next())) {
      log(*pName);
    }
  }

  // check each link reference
  acpListIterator<acpPackageTag> tags2(pPkg->m_tags);
  while ((pTag = tags2.next())) {
    switch (pTag->getTagType()) {
      case aTAG_PROPERTY:
      {
        acpTag_PROPERTY* pPropTag = (acpTag_PROPERTY*)pTag;
        // no string is okay, it's the default
        if (pPropTag->m_linkname.length() == 0) break;
        int nFound = findString(listLinkNames, pPropTag->m_linkname);
        if (!nFound) {
          acpString msg;
          msg += "In PROPERTY ";
          msg += (char*)pPropTag->m_name;
          msg += ", LINKNAME not recognized:  ";
          msg += (char*)pPropTag->m_linkname;
          log(msg);
          updateImportErrors(1);
        }
      }
    }
  }
}


/////////////////////////////////////////////////////////////////////

void acpRobotPackager::loadCUP(
  const char* pName,
  unsigned long* pSize,
  char** pBuff,
  aErr* pErr
)
{
  aErr err = aErrNone;
  unsigned long ulFileSize = 0;
  unsigned long ulBytesRead = 0;
  char buff[aFILE_NAMEMAXCHARS];

  *pBuff = NULL;
  aStringCopy(buff, pName);

  aFileRef myFile;  
  aFile_Open(m_ioRef,
	     buff,
	     aFileModeReadOnly,
	     aFileAreaObject,
	     &myFile,
	     &err);

  if (err == aErrNone)
    aFile_GetSize(m_ioRef,
		  myFile,
		  &ulFileSize,
		  &err);

  if (!ulFileSize) {
    err = aErrSize;
  }

  if ((err == aErrNone) && (ulFileSize > 0)) {
    *pBuff = 
      (char*)aMemAlloc(ulFileSize * sizeof(char));
    if (*pBuff)
      aFile_Read(m_ioRef,
		 myFile,    
		 *pBuff,
		 ulFileSize,
		 &ulBytesRead,
		 &err);
  }

  // if mess was made then clean it up
  if ((err != aErrNone) || (ulFileSize != ulBytesRead)) {
    if (ulFileSize != ulBytesRead)
      err = aErrSize;
    if (*pBuff != NULL)
      aMemFree(*pBuff);
    *pBuff = NULL;
    *pSize = 0;
  } else {
    *pSize = ulFileSize;
  }

  // dispose of file  
  aFile_Close(m_ioRef,
	      myFile,
	      NULL);

  // return status
  if (pErr)
    *pErr = err;
}


/////////////////////////////////////////////////////////////////////

void acpRobotPackager::compileTEA(
  const char* pName,
  char** ppCup,
  unsigned long* pulCupSize,
  aErr* pErr
)
{
  aErr err = aErrNone;
  aStreamRef source = NULL;
  aStreamRef result = NULL;
  aBool bCompile = aTrue;
  char outputfile[aFILE_NAMEMAXCHARS];
  char subpath[aFILE_NAMEMAXCHARS];
  char truepath[aFILE_NAMEMAXCHARS];

  if ((aStringLen(pName) >= aFILE_NAMEMAXCHARS)) {
    log(aSTEEP_FILENAME_LEN);
    bCompile = aFalse;
  }

  if ((bCompile == aTrue) &&
      aStream_CreateFileInput(m_ioRef, pName, aFileAreaUser, 
      			      &source, &err)) {
    log("Could not open file");
    bCompile = aFalse;
  }

  if (bCompile) {
    aString_GetFileRoot(outputfile, pName);
    aStringCat(outputfile, ".cup");
    aStringCopy(truepath, "../");
    aStringCat(truepath, txtFileAreaObject);
    getFilePath(subpath, pName);
    aStringCat(truepath, subpath);
    if (!osEnsureDirectory(truepath)) {
      log("Could not access or create directory");
      bCompile = aFalse;
    }
  }

  if ((bCompile == aTrue) &&
      aStream_CreateFileOutput(m_ioRef, outputfile, 
      			       aFileAreaObject, &result, &err)) {
    bCompile = aFalse;
  }

  // if the compilation fails the CUP file will have 0 size
  // (output stream is NULL since no need to be verbose)
  if (bCompile == aTrue)
    aSteep_Compile(m_steepRef,
    		   source, 
    		   pName, 
    		   fSteepGenerateCode, 
    		   result, 
    		   NULL,
    		   getLogStream(), 
    		   &err);

  // clean up
  if (source != NULL)
    aStream_Destroy(m_ioRef, source, NULL);
  if (result != NULL)
    aStream_Destroy(m_ioRef, result, NULL);
  
  if (bCompile) {
    loadCUP(outputfile, pulCupSize, ppCup, &err);
    if (pulCupSize == 0) err = aErrSize;
  }

  *pErr = err;
}
