/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRobotXMLReader.h                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Robot API script object and      //
//              supporting objects.                                //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//  Behaviors parametrize primitives.  They can also act as        //
//  containers for other sets of behaviors.                        //
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

#ifndef _acpRobotXMLReader_H_
#define _acpRobotXMLReader_H_

#include "aRobotOSExport.h"
#include "aErr.h"
#include "acpObject.h"
#include "acpList.h"
#include "acpValue.h"
#include "acpRobotBehaviorList.h"


#define	acpACTION_IN_ACTION	0x0001
#define	acpACTION_IN_BEHAVIOR	0x0002
#define	acpACTION_IN_PROPERTY	0x0004

/*
/////////////////////////////////////////////////////////////////////

class acpAction :
  public acpObject {
  
  public:
  				acpAction();
    virtual			~acpAction() {};

    virtual bool		execute(aStreamRef status) = 0;

#if 0
    virtual aErr		setProperty(const char* key,
    				            const char* pValue);
    virtual aErr		setProperty(const char* key,
    				            const int nValue);
    virtual aErr		setProperty(const char* key,
    				            const float fValue);
#endif

    virtual void		setRef(void* pRef)
				  { m_pRef = pRef; }

  protected:

    char			namebuff[250];
    char			valbuff[250];
    void*			m_pRef;
    acpValue			m_value;
};



/////////////////////////////////////////////////////////////////////

class acpActionParam :
  public acpAction {
  
  public:
  				acpActionParam();
    virtual			~acpActionParam();

    virtual bool		execute(aStreamRef status);
    virtual aErr		writeToStream(const aStreamRef stream) const
				  { return aErrNone; }

    void*			m_behavior;
};



/////////////////////////////////////////////////////////////////////

class acpActionBehavior :
  public acpAction {
  
  public:
  				acpActionBehavior();
    virtual			~acpActionBehavior();

    virtual bool		execute(aStreamRef status);
    virtual aErr		writeToStream(const aStreamRef stream) const
				  { return aErrNone; }

    void*			m_behavior;
    int				m_nEndStatus;
    acpList<acpValue>		m_listEndStatus;
};
*/


/////////////////////////////////////////////////////////////////////

class acpRobotXMLReader :
  public acpObject {

  public:
  				acpRobotXMLReader();
  
				acpRobotXMLReader(
				  acpRobotBehaviorList* pBehaviorList,
				  acpRobotInternal* pRobotInternal,
				  aStreamRef input,
				  aStreamRef status,
				  aStreamRef error);
  
    virtual			~acpRobotXMLReader();

    static aErr 		errProc(tkError error,
				        const unsigned int nLine,
				        const unsigned int nColumn,
				        const unsigned int nData,
				        const char* data[],
				        void* errProcRef);

    static aErr		 	sActStart(aXMLNodeRef node,
	     		                 const char* pKey,
	     	        	         void* vpRef);

    static aErr 		sActContent(aXMLNodeRef node,
					   const char* pKey,
	                 		   const aToken* pValue,
		                	   void* vpRef);

    static aErr 		sActSeekContent(aXMLNodeRef node,
					   const char* pKey,
	                 		   const aToken* pValue,
		                	   void* vpRef);

    static aErr 		sActEnd(aXMLNodeRef node,
		    		       aXMLNodeRef parent,
			      	       const char* pKey,
			  	       void* vpRef);

    aErr			xmlSeek(const char* pKey,
					acpValue::eType type,
				        aXMLNodeRef node);
  				
    aErr			xmlImport(aStreamRef xmlStream,
  					  aStreamRef status = NULL,
  					  aStreamRef error = NULL);

    bool			go();

    virtual aErr		writeToStream(const aStreamRef stream) const
				  { return aErrNone; }

  private:
  
    void			runtimeError();
    bool			checkHistory(int nFlags);

  protected:
  
    acpRobotInternal*		m_pRobotInternal;
    aIOLib			m_ioRef;
    aStreamRef			m_streamInput;
    aStreamRef			m_streamStatus;
    aStreamRef			m_streamError;    
    aXMLRef			m_xml;
    acpRobotBehavior*		m_pParseBehavior;
    acpRobotBehaviorList*	m_pBehaviorList;

    acpValue			m_valueSeek;
    const char*			m_cCacheName;
    const char*			m_cFileName;
    bool			m_bSeekResult;
    bool			m_bParseOkay;
    bool			m_bResult;
    int				m_nCtrlFlags;
    int				m_nSignFac;
};

#endif // _acpRobotXMLReader_H_
