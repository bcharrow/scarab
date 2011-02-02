/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: jBehavior.java                                            //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of a the BrainStem communication        //
//              object.                                            //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// Copyright 1994-2008. Acroname Inc.                              //
//                                                                 //
// This software is the Behavior of Acroname Inc.  Any             //
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

import java.io.*;

public class jBehavior 
  extends jObject
{
  private jRobot robot;
  private jObject primitive;


  /////////////////////////////////////////////////////////////////
  
  public jBehavior(
    jRobot robot,
    jObject primitive,
    String name
  ) throws Exception
  {
    super("behavior", name);
  
    this.robot = robot;
    this.primitive = primitive;

    int n = primitive.numProperties();

    // Create the storage for all the values up front for 
    // efficiency.  Otherwise, the values array would be
    // re-created and copied with each addProperty call.
    ensureValueArray(numProperties() + n);

    // Clone the primitive's properties, the classname and
    // object name don't get copied.
    for (int i = 0; i < n; i++) {
      jProperty primitiveProp = primitive.getProperty(i);
      if ((i != nClassNameIndex)
          && (i != nNameIndex)) {
        jProperty prop = new jProperty(primitiveProp);
//    System.out.println("prop " + prop.getName() + " index " + i);
        addProperty(prop);
      }
    }

  } // jBehavior constructor


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
    if (nPropIndex >= values.length)
      throw new Exception("property index out of range for " 
      			  + getClassName() 
      			  + " (" 
      			  + nPropIndex 
      			  + ") in setValue");
    values[nPropIndex] = value;
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
    if (nPropIndex >= values.length)
      throw new Exception("property index out of range for " 
      			  + getClassName() 
      			  + " (" 
      			  + nPropIndex 
      			  + ") in getValue");
    return values[nPropIndex];
  }


  /////////////////////////////////////////////////////////////////

  public int getPrimitiveIndex()
  {
    return primitive.getIndex();
  }


  /////////////////////////////////////////////////////////////////

  public void writeParam(
    DataOutputStream out, 
    int nIndex
  ) throws Exception
  {
    jProperty property = getProperty(nIndex);
    property.write(out, values[nIndex]);
  }


  /////////////////////////////////////////////////////////////////

  public void readParam(
    DataInputStream in, 
    int nIndex
  ) throws Exception
  {
    jProperty property = getProperty(nIndex);
    values[nIndex] = property.read(in);
  }


  /////////////////////////////////////////////////////////////////

  public void prepareCallbacks()
    throws Exception
  {
    // walk the list of set values and switch out 
    // callback objects with the integer identifiers
    for (int i = 0; i < values.length; i++) {
      if (values[i] != null) {
        jProperty prop = getProperty(i);
        if (prop.isCallback()) {

          // switch out the callback object with the 
          // unique index given when registered
          jCallback cb = (jCallback)values[i];
          int id = robot.registerCallback(cb);
          cb.finalize(id, this, robot);
          values[i] = new Integer(id);

        } // if a callback
      } // if not null
    } // for   
  } // jBehavior prepareCallbacks method

} // jBehavior class
