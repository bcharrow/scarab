/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: jRobot.java                                               //
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

import java.net.*;
import java.io.*;
import java.util.*;

public class jRobot extends jObject
{
  private jSettingsFile settings;
  private Socket socket;
  private Hashtable callbacks;
  private static int timeout = 1000;
  private static int nextCallbackID = 999;
  private Thread currentThread;
  

  /////////////////////////////////////////////////////////////////
 
  public jRobot() throws Exception
  {
    super("aRobot", "aRobot");

    timeout = 1000;
    callbacks = new Hashtable();
    settings = new jSettingsFile(new File("jRobot.config"));
 
    currentThread = Thread.currentThread();

    initialize();

  } // jRobot constructor



  /////////////////////////////////////////////////////////////////

  public void finalize()
  {
    try {
      if (socket != null) {
        out.write('X');
        out.flush(); /* fix submitted by Nicholas J. Santos (thanks!) */

        // sleep to allow the exit command across the wire
        // before destroying the socket
        currentThread.sleep(250);

	// now, blow away the socket
        socket.close();
      }
    }
    catch (Exception e) {
    }

  } // finish method



  /////////////////////////////////////////////////////////////////

  public jBehavior createBehavior(
    String primitiveName,
    String objectName) throws Exception
  {
    jObject primitive = findPrimitive(primitiveName);
    jBehavior behavior = new jBehavior(this, primitive, objectName);
    return behavior;

  } // createBehavior



  /////////////////////////////////////////////////////////////////

  public void queueBehavior(
    jBehavior behavior) throws Exception
  {
    // push the behavior as a data burst to the agent
    out.writeByte('B'); // send behavior gram
    out.writeShort(behavior.getPrimitiveIndex());
    String name = behavior.getName();
    out.writeBytes(name);
    out.writeByte(0);

    // switch out callbacks objects with identifiers
    behavior.prepareCallbacks();
 
    // count the number of values that have been set
    // skipping the name and classname
    int n = 0;
    int i;
    for (i = 0; i < behavior.numProperties(); i++) {
      if ((i != nNameIndex) 
          && (i != nClassNameIndex)
      	  && (behavior.getValue(i) != null))
        n++;
    }

    // write the number of values that have been set
    out.writeShort(n);
  
    // then write the actual property values
    for (i = 0; i < behavior.numProperties(); i++) {
      if ((i != nNameIndex) 
          && (i != nClassNameIndex)
      	  && (behavior.getValue(i) != null)) {

      	jProperty prop = behavior.getProperty(i);

        out.writeShort(i); // property index
        behavior.writeParam(out, i);
      }
    }

    out.flush();
 
    behavior.setValue("unique-id", new Integer(readShort()));

  } // queueBehavior



  /////////////////////////////////////////////////////////////////

  public void handleCallbacks(
    long nMSec
  ) throws Exception
  {
    long start = 0;
    long now = 0;

    do {
      out.writeByte('C'); // request callbacks
      out.flush();
      
      int numCB = readShort();
      
      // collect a list of callbacks that are being reported
      jCallback done[] = null;
      if (numCB > 0)
        done = new jCallback[numCB];
      for (int i = 0; i < numCB; i++) {

        // find the index of the callback
        Integer cbIndex = new Integer(readInt());
        jCallback cb = (jCallback)callbacks.remove(cbIndex);
        if (cb == null)
          throw new Exception("callback sequence error: cbIndex = " + cbIndex);

        // see how many properties get updated
        short nPropChanges = readShort();
        for (int j = 0; j < nPropChanges; j++) {
          int propIndex = readShort();
          cb.behavior.readParam(in, propIndex);
        }

        // store the callback till we are done with this 
        // transaction
        done[i] = cb;
      }

      // wait till the "C" transaction is done to
      // call the callbacks
      if (done != null) {
        for (int i = 0; i < numCB; i++)
          done[i].call();

      // sleep a bit to avoid swamping things
      } else {
        try {
          currentThread.sleep(5);
        } catch (Exception e) {}
      }

      if (start == 0)
        start = System.currentTimeMillis();
      
      now = System.currentTimeMillis();

    } while (now < (start + nMSec));

  } // handleCallbacks


  /////////////////////////////////////////////////////////////////
 
  private jObject findPrimitive (
    String key
  ) throws Exception
  {
    jObject primitive = null;
    if (key != null) {
      primitive = getSubObject("primitive", key);
      if (primitive == null)
        throw new Exception("invalid primitive for jRobot: " + key);
    }

    return primitive;

  } // findPrimitives



  /////////////////////////////////////////////////////////////////
 
  private jObject findPrimitive (
    int nIndex
  ) throws Exception
  {     
    for (Enumeration e = subObjects.elements(); 
    	 e.hasMoreElements() ;) {
      jObject prim = (jObject)e.nextElement();
      if ((prim.getIndex() == nIndex)
          && prim.getClassName().equals("primitive"))
        return prim;
    }

    throw new Exception("primitive at index " 
    			+ nIndex + " not found");

  } // findPrimitive



  /////////////////////////////////////////////////////////////////
 
  private jProperty findProperty (
    String key,
    boolean bRead,
    boolean bWrite
  ) throws Exception
  {
    jProperty property = null;
    if (key != null) {
      property = (jProperty)properties.get(key);
      if (property == null)
        throw new Exception("invalid property for Garcia: " 
        		    + key);
      if (bWrite && !property.canWrite())
        throw new Exception("write-only property for Garcia: " 
        		    + key);
      if (bRead && !property.canRead())
        throw new Exception("read-only property for Garcia: " 
        		    + key);
    }

    return property;

  } // findProperty method



  /////////////////////////////////////////////////////////////////
 
  private void initialize() throws Exception
  {
    System.out.println("initializing");

    // get the IP address
    int port = settings.getInt("robot_port", 8008);
    System.out.println("got the port");
    String address = settings.getString("robot_address", 
    					"127.0.0.1");

    try {
      System.out.println("opening socket on " 
      			 + address + " port " + port);
      socket = new Socket(address, port);
      socket.setSoTimeout(timeout);
      socket.setTcpNoDelay(true);
      socket.setSoLinger(false, 0);
      socket.setKeepAlive(false);
 
      out = new DataOutputStream(
      	      new BufferedOutputStream(socket.getOutputStream(), 
      	      			       1024));
      in = new DataInputStream(
      	     new BufferedInputStream(socket.getInputStream()));

      ack();
      sync();

    } catch (Exception e) {
      throw new Exception("Unable to create connection to aRobot API: " 
      			  + e.toString());
    }

  } // jRobot initialize method



  /////////////////////////////////////////////////////////////////

  private void ack() throws Exception
  {
    out.writeByte('A'); // say hello
    out.flush();

    int response = nextByte();
    if (response == -1)
      System.out.println("no ack");
    else if (response == 'A') {
      nAddr = readInt();
    } else if (response == 'B') {
      throw new Exception("Agent is Busy");
    }
  } // ack method



  /////////////////////////////////////////////////////////////////

  private int nextByte() throws Exception
  {
    int c;
    long start = 0;
    long now = 0;

    do {
      c = in.read();
      if (c != -1)
        return c;
      if (start == 0)
        start = System.currentTimeMillis();
      else
        now = System.currentTimeMillis();

    } while (now < (start + timeout));
 
    throw new Exception("timeout getting data from aGarciaAgent");

  } // nextByte



  /////////////////////////////////////////////////////////////////

  private short readShort() throws Exception
  {
    int val = (int)(nextByte() << 8);
    val += (int)nextByte();

    return (short)val;

  } // readShort method



  /////////////////////////////////////////////////////////////////

  private int readInt() throws Exception
  {
    int val;
    int i;

    i = nextByte();
    val = (i << 24);
    i = nextByte();
    val += (i << 16);
    i = nextByte();
    val += (i << 8);
    i = nextByte();
    val += i;

    return val;

  } // readInt method



  /////////////////////////////////////////////////////////////////

  private String readString() throws Exception
  {
    String val = "";

    int c = 0;
    do {
      c = nextByte();
      if (c > 0)
        val += (char)c;
    } while (c > 0);
      
    return val;

  } // readString method



  /////////////////////////////////////////////////////////////////

  private void sync() throws Exception
  {
    int i;

    System.out.println("synchronizing with agent");

    // get the properties
    out.writeByte('P');
    out.writeInt(nAddr);
    out.flush();

    int nProperties = readShort();
    for (i = 0; i < nProperties; i++) {
      String name = readString();
      int type = readInt();
      if ((i != nNameIndex) && (i != nClassNameIndex)) {
        jProperty property = new jProperty(name, type);
        addProperty(property);
      }
    }

    // get the subobjects
    out.writeByte('O');
    out.writeInt(nAddr);
    out.flush();

    int nSubObjects = readShort();
    for (i = 0; i < nSubObjects; i++) {
      String classname = readString();
      String name = readString();
      int address = readInt();

      // building the object below will create the two
      // basic properties of name and classname in the
      // subobjects
      addSubObject(new jObject(out, in, classname, 
      			       name, address));
    }

    // now find out about each subobject
    for (i = 0; i < nSubObjects; i++) {

      jObject subObject = getSubObject(i);

      out.writeByte('P');
      out.writeInt(subObject.getAddr());
      out.flush();

      nProperties = readShort();
      for (int j = 0; j < nProperties; j++) {
        String name = readString();
        int type = readInt();
        // skip the name and classname for the sub-objects
        // as they are already in place from creation
        if ((j != nNameIndex) && (j != nClassNameIndex)) {
          jProperty property = new jProperty(name, type);
          subObject.addProperty(property);
        }
      }
    }

  } // sync method



  /////////////////////////////////////////////////////////////////

  public int registerCallback(
    jCallback callback)
  {
    int id = nextCallbackID++;
    callbacks.put(new Integer(id), callback);
    return id;

  } // registerCallback method



  /////////////////////////////////////////////////////////////////
  // This is temporary, please don't call this directly, use the 
  // jObject named camera instead!

    //  public void setObjectFloatProperty(
    //    int propertyIndex,
    //    float value
    //  ) throws Exception
    //  {
    //     out.writeByte('T'); // request camera move (temporary)
    //     out.writeShort(propertyIndex);
    //     out.writeFloat(value);

    //  } // setObjectFloatProperty method

} // jRobot class
