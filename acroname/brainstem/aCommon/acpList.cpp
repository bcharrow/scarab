/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* file: acpList.cpp 		  		                   */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* description: Non-intrusive list template class implementation.  */
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

#include "aAssert.h"
#include "acpList.h"


#ifdef aDEBUG
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listbase hasIntegrity method
 */

bool _listbase::hasIntegrity() const
{
  if (m_pHead && (m_pHead->m_pPrev))
    return false;

  if (m_pTail && (m_pTail->m_pNext))
    return false;

  return true;

} // hasIntegrity method
#endif /* aDEBUG */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listbase toFront method
 */

void _listbase::toFront(void* pObject)
{
  _listnode* n = new _listnode(pObject);

  if (m_pHead)
    m_pHead->m_pPrev = n;
  n->m_pNext = m_pHead;
  n->m_pPrev = 0L;
  m_pHead = n;
  if (!m_pTail)
    m_pTail = n;
  
  m_nLength++;

  aAssert(hasIntegrity());

} // toFront method


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listbase toBack method
 */

void _listbase::toBack(void* pObject)
{ 
  _listnode* n = new _listnode(pObject);

  if (m_pTail)
    m_pTail->m_pNext = n;
  n->m_pPrev = m_pTail;
  n->m_pNext = 0L;
  m_pTail = n;
  if (!m_pHead)
    m_pHead = n;

  m_nLength++;

  aAssert(hasIntegrity());

} // toBack method


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listbase removeHead method
 */

void* _listbase::removeHead()
{
  _listnode* n = m_pHead;

  if (!n)
    return 0L;

  if (n->m_pNext)
    n->m_pNext->m_pPrev = 0L;
  m_pHead = n->m_pNext;

  if (!m_pHead)
    m_pTail = 0L;

  m_nLength--;

  void* p = n->m_pObject;

  delete(n);

  aAssert(hasIntegrity());

  return p;

} // removeHead method


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listbase removeTail method
 */

void* _listbase::removeTail()
{
  _listnode* n = m_pTail;

  if (!n)
    return 0L;

  if (n->m_pPrev)
    n->m_pPrev->m_pNext = 0L;
  m_pTail = n->m_pPrev;

  if (!m_pTail)
    m_pHead = 0L;

  m_nLength--;

  void* p = n->m_pObject;

  delete(n);

  aAssert(hasIntegrity());

  return p;

} // removeTail method


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listbase array operator method
 */

void* _listbase::operator[](const int i) const 
{
  _listnode* n;
  
  aAssert(i < m_nLength);

  // look from back to front if index is over half length
  if (i > (m_nLength >> 1)) {
    int cur = m_nLength - 1;
    n = m_pTail;
    while (n && (i != cur--))
      n = n->m_pPrev;
  } else {
    int cur = 0;
    n = m_pHead;
    while (n && (i != cur++))
      n = n->m_pNext;
  }

  return n ? n->m_pObject : 0L;

} // operator[]


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listbase remove method
 *
 * Pulls the passed in object from the list and returns it.  If the
 * item can't be found, a NULL pointer is returned.
 */

void* _listbase::remove(void* pObject)
{
  _listnode* p = NULL;
  _listnode* t = m_pHead;

  // walk the list looking for the matching object
  while (t && (t->m_pObject != pObject)) {
    p = t;
    t = t->m_pNext;
  }

  if (t) {

    // splice the removed node from the list
    //
    // need these cases
    // 0 1 r 3 4:  p = 1, t = 2
    // r 1 2 3 4:  p = NULL, t = 0
    // 0 1 2 3 r:  p = 3, t = 4
    // r        :  p = NULL, t = 0
    // r 1      :  p = NULL, t = 0
    // 0 r      :  p = 0, t = 1

    if (p)
      p->m_pNext = t->m_pNext;
    else
      m_pHead = t->m_pNext;
    if (t->m_pNext)
      t->m_pNext->m_pPrev = p;
    
    if (t == m_pTail)
      m_pTail = (p) ? p : NULL;

    // record the item being removed
    m_nLength--;

    aAssert(hasIntegrity());

    // cache the object pointer
    void* o = t->m_pObject;

#ifdef aDEBUG
    // slam these for debug purposes 
    t->m_pNext = NULL;
    t->m_pPrev = NULL;
    t->m_pObject = NULL;
#endif // aDEBUG

    // get rid of the node
    delete t;

    return o;
  } else 
    return NULL;

} // remove method


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * _listiter next method
 */

void* _listiter::next()
{
  void* p;

  aAssert(m_pList->hasIntegrity());

  if (m_pCurrent) {
    p = m_pCurrent->m_pObject;
    m_pCurrent = m_pCurrent->m_pNext;
  } else
    p = 0L;

  return p;

} // next method

