/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: acpList.h 		  		                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Non-intrusive list template class definition.      */
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

#ifndef _acpList_H_
#define _acpList_H_

#include "aUtil.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class aUTIL_EXPORT _listnode {
  public:
  			_listnode(
			  void* pObject) : 
  			  m_pObject(pObject),
			  m_pNext(NULL),
			  m_pPrev(NULL) 
			  {}
    virtual		~_listnode() {}

    void*		m_pObject;

    _listnode*		m_pNext;
    _listnode*		m_pPrev;
 
  friend class _listbase;
  friend class _listiter;
};



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class aUTIL_EXPORT _listbase {
  public:
  			_listbase() :
                          m_nLength(0),
  			  m_pHead(0L),
  			  m_pTail(0L) {}
    virtual		~_listbase() {}

    const int		length() const 
    			  {return m_nLength;}

  protected:
    void		toFront(void* pObject);
    void		toBack(void* pObject);

    void*		head() const
    			  {return m_pHead ? m_pHead->m_pObject : 0L;}
    void*		removeHead();

    void*		tail() const
    			  {return m_pTail ? m_pTail->m_pObject : 0L;}
    void*		removeTail();
    void*		operator[](const int i) const;
    void*		remove(void* pObject);

    int			m_nLength;

    _listnode*		m_pHead;
    _listnode*		m_pTail;
  
private:
#ifdef aDEBUG
    bool		hasIntegrity() const;
#endif /* aDEBUG */

  friend class _listiter;
};



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class aUTIL_EXPORT _listiter {
  public:
  			_listiter(const _listbase* l) :
  			  m_pList(l),
  			  m_pCurrent(l->m_pHead) {}
    virtual		~_listiter() {}

    void		reset() 
    			  {m_pCurrent = m_pList->m_pHead;}

  protected:
    void*		next();

  private:
    const _listbase*	m_pList;
    _listnode*		m_pCurrent;
};



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

template<class T>
class acpList : public _listbase
{
  public:
			acpList<T>() {}
    virtual		~acpList<T>()
    			  {
  			    while (m_pHead) {
    			      _listnode* n = m_pHead;
    			      m_pHead = m_pHead->m_pNext;
    			      T* p = (T*)n->m_pObject;
    			      delete p;
    			      delete n;
    			    }
  			  }

    void   		add(const T* x)
    			  {toBack((void*)x);}
    void   		addToHead(const T* x)
    			  {toFront((void*)x);}
    void   		addToTail(const T* x)
    			  {toBack((void*)x);}
    T*			head() const
    			  {return (T*)_listbase::head();}
    T*			removeHead()
    			  {return (T*)_listbase::removeHead();}
    T*			tail() const
    			  {return (T*)_listbase::tail();}
    T*			removeTail()
    			  {return (T*)_listbase::removeTail();}
    T*			operator[](const int i) const
    			  {return (T*)_listbase::operator[](i);}
    T*			remove(const T* x) 
    			  {return (T*)_listbase::remove((void*)x);}
    const bool		isEmpty() const {return m_nLength == 0;}
};



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

template<class T>
class acpListIterator : public _listiter
{
  public:
			acpListIterator(const acpList<T>& list) : 
			  _listiter(&list) {}
    virtual		~acpListIterator() {}

    T*			next() 
    			  {return (T*)_listiter::next();}
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define aLISTITERATE(t, l, v)                                       \
                                                                    \
t* v;                                                               \
acpListIterator<t> _alist(l);                                       \
while ((v = _alist.next()))

#endif /* _acpList_H_ */
