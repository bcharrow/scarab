/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpProperty.h                                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Garcia API property object.      //
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

#ifndef _acpProperty_H_
#define _acpProperty_H_

#include "aIO.h"
#include "acpValue.h"

typedef int aPROPERTY_FLAGS;

// property flags
#define aPROPERTY_FLAG_INT	0x00000001
#define aPROPERTY_FLAG_FLOAT	0x00000002
#define aPROPERTY_FLAG_STRING   0x00000004
#define aPROPERTY_FLAG_BOOL	0x00000008
#define aPROPERTY_FLAG_CALLBACK	0x00000010
#define aPROPERTY_FLAG_VOIDPTR	0x00000020
#define aPROPERTY_FLAG_OBJECT	0x00000040
#define aPROPERTY_FLAG_USERBIT	0x20000000
#define aPROPERTY_FLAG_WRITE	0x40000000
#define aPROPERTY_FLAG_READ	0x80000000
#define aPROPERTY_TYPE_MASK	0x0000007F

typedef aErr (*propertyEnumProc)(const char* pName,
				 const int nIndex,
				 aPROPERTY_FLAGS flags,
				 void* vpRef);

class acpObject;

class acpProperty {

  public:
  			acpProperty(
  			  const char* pName,
  			  const aPROPERTY_FLAGS ucType,
  			  const char* pDescription = NULL);
  virtual		~acpProperty();

    char*		getName() const
                	  { return m_pName; }
    aPROPERTY_FLAGS	getTypeFlags() const
                	  { return m_typeFlags; }

    aErr		readValue(aStreamRef stream,
    				  acpValue* pValue);

    virtual void	setValue(const acpValue* pValue);
    virtual void	getValue(const acpObject* pObject,
    				 acpValue* pValue);

    const char*         getDescription() 
                          { return (m_pDescription) ?
                                      m_pDescription : "none";}

  private:
    void		setIndex(
    			  const int nIndex)
    			  { m_nIndex = nIndex; }
    char*		m_pName;
    int			m_nIndex;
    aPROPERTY_FLAGS	m_typeFlags;
    char*		m_pDescription;
  
  friend class acpObject;
};

#endif // _acpProperty_H_
