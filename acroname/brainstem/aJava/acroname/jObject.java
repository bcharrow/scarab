/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: jObject.java                                              //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the basic Acroname object base       //
//              class.                                             //
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

package acroname;

import java.util.*;
import java.io.*;

public class jObject
{
  protected DataOutputStream out;
  protected DataInputStream in;
  protected int nIndex = 0;
  protected int nAddr = 0;
  protected Hashtable subObjects;
  protected Hashtable properties;
  protected Object values[];
  protected int nClassNameIndex;
  protected int nNameIndex;


  /////////////////////////////////////////////////////////////////

  protected jObject(
    String classname,
    String name
  )
  {
    initialize(classname, name);
  }


  /////////////////////////////////////////////////////////////////

  public jObject(
    DataOutputStream out,
    DataInputStream in,
    String classname,
    String name
  )
  {
    initialize(classname, name);

    this.in = in;
    this.out = out;
  }


  /////////////////////////////////////////////////////////////////

  public jObject(
    DataOutputStream out,
    DataInputStream in,
    String classname,
    String name,
    int addr
  )
  {
    this(classname, name);

    this.in = in;
    this.out = out;

    this.nAddr = addr;
  }


  /////////////////////////////////////////////////////////////////

  private void initialize(
    String classname,
    String name
  )
  {
    // pre-allocate storage for the classname and name
    values = new Object[2];

    subObjects = new Hashtable();
    properties = new Hashtable();

    nClassNameIndex = addProperty(
    		new jProperty("classname", 
    			      jProperty.aPROPERTY_FLAG_STRING |
    			      jProperty.aPROPERTY_FLAG_READ), 
    			      classname);

    nNameIndex = addProperty(
    		new jProperty("name", 
    			      jProperty.aPROPERTY_FLAG_STRING |
    			      jProperty.aPROPERTY_FLAG_READ), 
    			      name);
  }
  

  /////////////////////////////////////////////////////////////////

  public int numProperties()
  {
    return properties.size();
  }


  /////////////////////////////////////////////////////////////////

  protected void ensureValueArray(
    int nSize
  )
  {
    // add space in the value array for the new property if
    // if needed
    if (nSize > values.length) {
      Object newVals[] = new Object[nSize];
      for (int i = 0; i < values.length; i++)
        newVals[i] = values[i];
      values = newVals;
    }
  }


  /////////////////////////////////////////////////////////////////

  public int addProperty(
    jProperty property
  )
  {
    // the index will be the highest index
    int index = numProperties();

    ensureValueArray(numProperties() + 1);
  
    property.setIndex(index);
 
    properties.put(property.getName(), property);
 
    return index;
  }


  /////////////////////////////////////////////////////////////////

  public int addProperty(
    jProperty property,
    Object defaultValue
  )
  {
    // add in the property
    int index = addProperty(property);

    // set the default
    values[index] = defaultValue;
 
    return index;
  }


  /////////////////////////////////////////////////////////////////

  public void addSubObject(
    jObject subObject
  )
  {
    int index = subObjects.size();
    subObject.setIndex(index);
    subObjects.put(subObject.getName(), subObject);
  }


  /////////////////////////////////////////////////////////////////

  public String getName()
  {
    return (String)values[nNameIndex];
  }


  /////////////////////////////////////////////////////////////////

  public String getClassName()
  {
    return (String)values[nClassNameIndex];
  }


  /////////////////////////////////////////////////////////////////

  public void setIndex(int nIndex)
  {
    this.nIndex = nIndex;
  }


  /////////////////////////////////////////////////////////////////

  public int getIndex()
  {
    return nIndex;
  }


  /////////////////////////////////////////////////////////////////

  public int getAddr()
  {
    return nAddr;
  }



  /////////////////////////////////////////////////////////////////

  public jObject getSubObject(
    String classname,
    String name
  ) throws Exception
  {
    for (Enumeration e = subObjects.elements(); 
    	 e.hasMoreElements() ;) {
      jObject obj = (jObject)e.nextElement();
      if ((obj.getClassName().equals(classname)) 
          && (obj.getName().equals(name)))
        return obj; 
    }
    throw new Exception("subobject not found for index " + nIndex);
  }



  /////////////////////////////////////////////////////////////////

  public jObject getSubObject(
    int nIndex
  )
    throws Exception
  {
    for (Enumeration e = subObjects.elements(); 
    	 e.hasMoreElements() ;) {
      jObject obj = (jObject)e.nextElement();
      if (obj.getIndex() == nIndex)
        return obj; 
    }
    throw new Exception("subobject not found for index " + nIndex);
  }


  /////////////////////////////////////////////////////////////////

  public int getPropertyIndex(
    String propName
  ) throws Exception
  {
    jProperty property = (jProperty)properties.get(propName);
    if (property == null)
      throw new Exception("invalid property for " + getName() 
      			  + " object named " + propName);
    return property.getIndex();
  }


  /////////////////////////////////////////////////////////////////

  public jProperty getProperty(
    int nIndex
  ) throws Exception
  {
    for (Enumeration e = properties.elements(); 
    	 e.hasMoreElements() ;) {
      jProperty prop = (jProperty)e.nextElement();
      if (prop.getIndex() == nIndex)
        return prop; 
    }
    throw new Exception("unable to find \"" 
    			+ getName() 
    			+ "\" object property with index " 
    			+ nIndex);
  }


  /////////////////////////////////////////////////////////////////

  public void setValue(
    String propName,
    Object value
  ) throws Exception
  {
    setValue(getPropertyIndex(propName), value);
  }


  /////////////////////////////////////////////////////////////////

  public void setValue(
    int nPropIndex,
    Object value
  ) throws Exception
  {
    jProperty prop = getProperty(nPropIndex);

    prop.send(this, value, out);
  }


  /////////////////////////////////////////////////////////////////

  public Object getValue(
    String propName
  ) throws Exception
  {
    return getValue(getPropertyIndex(propName));
  }


  /////////////////////////////////////////////////////////////////

  public Object getValue(
    int nPropIndex
  ) throws Exception
  {
    jProperty prop = getProperty(nPropIndex);
  
    prop.request(this, out);

    return prop.read(in);
  }


} // jObject class
