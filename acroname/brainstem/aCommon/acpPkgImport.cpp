/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpPkgImport.cpp 	 		                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of data package importer.           //
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

#include "aStream_STDIO_Console.h"
#include "aVersion.h"
#include "aUtil.h"

#include "acpException.h"
#include "acpPkgImport.h"



/////////////////////////////////////////////////////////////////////

acpPkgImport::acpPkgImport(
  aIOLib ioRef,
  aStreamRef logStream,
  aStreamRef input,
  acpPackage* pPackage
) :
  m_nCurrentOwner(0),
  m_nInstanceFlags(0),
  m_pPackage(pPackage),
  m_ioRef(ioRef),
  m_logStream(logStream),
  m_input(input),
  m_nImportErrors(0),
  m_nPackageNesting(0),
  m_namedTags(NULL)
{
}


/////////////////////////////////////////////////////////////////////

acpPkgImport::~acpPkgImport()
{
}


/////////////////////////////////////////////////////////////////////

void acpPkgImport::import(
  aStreamRef input
)
{
  aXMLCallbacks cb = {
    sStart,
    sContent,
    sEnd,
    this
  };

  aErr err;

  if (aSymbolTable_Create(m_ioRef, &m_namedTags, &err))
    throw acpException(err, "unable to create named tag symbol table");

  try {
    if (aXML_Create(m_ioRef, m_input, xmlErr, this, &m_xml, &err))
      throw acpException(err, "error importing xml");

    if (aXML_Traverse(m_ioRef, m_xml, &cb, &err))
      throw acpException(err, "error traversing xml");

    if (aXML_Destroy(m_ioRef, m_xml, &err))
      throw acpException(err, "destroying xml");
  
  } catch (const acpException& exception) {
    m_nImportErrors++;
    error(exception.msg());
  }

  if (m_namedTags) {
    aSymbolTable_Destroy(m_ioRef, m_namedTags, NULL);
    m_namedTags = NULL;  	
  }
}



/////////////////////////////////////////////////////////////////////
    
void acpPkgImport::error(
  const char* pMsg)
{
  aStream_WriteLine(m_ioRef, m_logStream, pMsg, NULL);
}



/////////////////////////////////////////////////////////////////////
    
void acpPkgImport::log(
  const char* pMsg)
{
  aStream_WriteLine(m_ioRef, m_logStream, pMsg, NULL);
}

  

/////////////////////////////////////////////////////////////////////

void acpPkgImport::addTag(
  acpPkgXML* pXMLOrigin,
  acpPackageTag* pTag
)
{
  // when objects are named, insert them into the symbol table
  // for named objects
  const char* pObjectName = pXMLOrigin->getObjectName();
  if (pObjectName) {
    aErr err;
    aSymbolTable_Insert(m_ioRef,
    			m_namedTags,
    			pObjectName,
			(void*)pTag,
			symDontDelete,
			NULL,
			&err);
  }
}


/////////////////////////////////////////////////////////////////////

acpPackageTag* acpPkgImport::getNamedTag(
  const char* pName) 
{
  // search the symbol table for named objects to see if the 
  // tag can be found
  aErr e;
  void* vp = NULL;
  if (!aSymbolTable_Find(m_ioRef, m_namedTags, pName, &vp, &e))
    return((acpPackageTag*)vp);
  else
    return NULL;
}


/////////////////////////////////////////////////////////////////////

aErr acpPkgImport::xmlErr(
  tkError error,
  const unsigned int nLine,
  const unsigned int nColumn,
  const unsigned int nData,
  const char* data[],
  void* errProcRef
)
{
  aErr err = aErrNone;
  acpPkgImport* pImport = (acpPkgImport*)errProcRef;
  char msg[100];
  char num[10];

  pImport->error("package import error:");
 
  /* show the error location */
  
  /* the filename is always the first data parameter */
  if ((err == aErrNone) && (nData > 0)) {
    aStringCopy(msg, " file: ");
    aStringCat(msg, data[0]);
    pImport->error(msg);
  }

  if (err == aErrNone) {        
    aStringCopy(msg, " line: ");
    aStringFromInt(num, nLine);
    aStringCat(msg, num);
    pImport->error(msg);
  }
    
  if (err == aErrNone) {
    aStringCopy(msg, " character: ");
    aStringFromInt(num, nColumn);
    aStringCat(msg, num);
    pImport->error(msg);
  }

  /* show the actual error */  
  if (err == aErrNone) {
    aStringCopy(msg, " ");

    switch(error) {

    case tkErrUntermCmnt:
      aStringCat(msg, "Unterminated Comment");
      break;

    case tkErrIncludeNotFnd:
      aStringCat(msg, "Include File not Found: ");
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkErrDuplicateDefine:
      aStringCat(msg, "Symbol Already Defined: ");
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkErrBadArgumentList:
      aStringCat(msg, "Bad Argument List: ");
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;
      
    case tkErrXMLMissingClose:
      aStringCat(msg, "Missing XML Tag Close");
      break;
      
    case tagTokenError:
      aStringCat(msg, "Bad Token Found");
      if (nData > 1) {
        aStringCat(msg, ": ");
        aStringCat(msg, data[1]);
      }
      break;
      
    case tagFileError:
      aStringCat(msg, "File Error");
      if (nData > 1) {
        aStringCat(msg, ": ");
        aStringCat(msg, data[1]);
      }
      break;

#if 0
    case tkFileNameType:
      aStringCat(msg, "File Name Expected");
      break;
    
    case tkNameType:
      aStringCat(msg, "Entity Name Expected");
      break;

    case tkIntType:
      aStringCat(msg, "Integer Expected");
      break;

    case tkMissingParent:
      aStringCat(msg, "Named Parent Not Found: ");
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    case tkBadFloat:
      aStringCat(msg, "Bad Floating Point Value");
      break;

    case tkBadVector:
      aStringCat(msg, "Bad Vector Value");
      break;

    case tkBadRotation:
      aStringCat(msg, "Bad Rotation Value");
      break;
#endif

    default:
      if (nData > 1)
        aStringCat(msg, data[1]);
      break;

    } // switch  

    pImport->error(msg);
  }

  // increment error count
  pImport->m_nImportErrors++;

  return err;
}


/////////////////////////////////////////////////////////////////////

aErr acpPkgImport::sStart(
  aXMLNodeRef node,
  const char* pKey,
  void* vpRef)
{
  aErr err = aErrNone;
  acpPkgImport* pImport = (acpPkgImport*)vpRef;

  bool bPackageTag = aStringCompare(pKey, "PACKAGE") == 0;

  // make sure we are not nesting
  if (bPackageTag) {
    if (pImport->m_nPackageNesting > 0)
      pImport->parseError("PACKAGE tags cannot be nested");
    pImport->m_nPackageNesting++;
  }

  // skip all content if we are not in a package
  if (pImport->m_nPackageNesting == 1) {
    acpPkgXML* pTag = NULL;
    acpListIterator<acpPkgXML> tags(pImport->m_tags);
    while ((pTag = tags.next())) {
      if (!aStringCompare(pTag->tagName(), pKey)) {
        // build an empty tag and stick it on the stack
        acpPkgXML* pParent = pImport->m_stack.head();
        pImport->m_stack.addToHead(pTag->clone(pParent));
        break;
      }
    }

    // report if tag was not recognized
    if (!pTag) {
      acpString msg("invalid XML tag ");
      msg += pKey;
      pImport->parseError(msg);
    }
  }
  return err;
}


/////////////////////////////////////////////////////////////////////

aErr acpPkgImport::sContent(
  aXMLNodeRef node,
  const char* pKey,
  const aToken* pValue,
  void* vpRef
)
{
  aErr err = aErrNone;
  acpPkgImport* pImport = (acpPkgImport*)vpRef;

  if (pImport->m_nPackageNesting == 1) {
    acpPkgXML* pTag = pImport->m_stack.head();
    if (pTag)
      pTag->addToken(pValue);
  }

  return err;
}


/////////////////////////////////////////////////////////////////////

aErr acpPkgImport::sEnd(
  aXMLNodeRef node,
  aXMLNodeRef parent,
  const char* pKey,
  void* vpRef
)
{
  aErr err = aErrNone;
  acpPkgImport* pImport = (acpPkgImport*)vpRef;

  if (pImport->m_nPackageNesting == 1) {
    acpPkgXML* pTag = pImport->m_stack.head();

    if (pTag) {

      if (!aStringCompare(pKey, pTag->tagName())) {
        pImport->m_stack.removeHead();
        pTag->end();

        // add the tag to the parent
        acpPkgXML* pParent = pImport->m_stack.head();
        bool bTagAccepted = false;
        if (pParent) {
          bTagAccepted = pParent->addTag(pTag);
          if (bTagAccepted) {
            pParent->addChild(pTag);
          } else {
            acpString msg(pTag->tagName());
            msg += " not accepted as a tag for ";
            msg += pParent->tagName();
            pImport->parseError(msg);
          }
        }

        if (!bTagAccepted)
          delete pTag;
      }
    }
  }

  if (!aStringCompare(pKey, "PACKAGE"))
    pImport->m_nPackageNesting--;

  return err;
}


/////////////////////////////////////////////////////////////////////

void acpPkgImport::parseError(
  const char* pMsg
)
{
  acpString msg("parse error: ");
  msg += pMsg;
  error(msg);
}
