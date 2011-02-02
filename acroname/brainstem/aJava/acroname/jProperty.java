/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: jProperty.java                                            //
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

import java.io.*;

public class jProperty
{
  private String name;
  private int nIndex = -1;
  private int nType;
  
  static final int aPROPERTY_FLAG_INT =		0x00000001;
  static final int aPROPERTY_FLAG_FLOAT =	0x00000002;
  static final int aPROPERTY_FLAG_STRING =	0x00000004;
  static final int aPROPERTY_FLAG_BOOL =	0x00000008;
  static final int aPROPERTY_FLAG_CALLBACK =	0x00000010;
  static final int aPROPERTY_FLAG_VOIDPTR =	0x00000020;
  static final int aPROPERTY_FLAG_OBJECT =	0x00000040;
  static final int aPROPERTY_FLAG_USERBIT =	0x20000000;
  static final int aPROPERTY_FLAG_WRITE =	0x40000000;
  static final int aPROPERTY_FLAG_READ =	0x80000000;
  static final int aPROPERTY_TYPE_MASK =	0x0000007F;
  
  private int timeout = 1000;

  /////////////////////////////////////////////////////////////////
  
  public jProperty (
    String name,
    int nType
  )
  {
    this.name = name;
    this.nType = nType;

  } // jProperty constructor


  /////////////////////////////////////////////////////////////////
  
  public jProperty (
    jProperty prop
  )
  {
    this.name = prop.name;
    this.nIndex = prop.nIndex;
    this.nType = prop.nType;

  } // jProperty constructor


  /////////////////////////////////////////////////////////////////
  
  public boolean canRead()
  {
    return((nType & aPROPERTY_FLAG_READ) != 0);

  } // canRead method


  /////////////////////////////////////////////////////////////////
  
  public boolean canWrite()
  {
    return((nType & aPROPERTY_FLAG_WRITE) != 0);

  } // canWrite method


  /////////////////////////////////////////////////////////////////
  
  public String getName()
  {
    return(name);

  } // getName method


  /////////////////////////////////////////////////////////////////
  
  public int getIndex()
  {
    return(nIndex);

  } // getIndex method


  /////////////////////////////////////////////////////////////////
  
  public void setIndex(
    int nIndex
  )
  {
    this.nIndex = nIndex;

  } // setIndex method



  /////////////////////////////////////////////////////////////////
  
  public boolean isCallback()
  {
    return((nType & aPROPERTY_FLAG_CALLBACK) != 0);

  } // isCallback method



  /////////////////////////////////////////////////////////////////

  private char nextByte(DataInputStream in) throws Exception
  {
    int c;
    long start = 0;
    long now = 0;

    do {
      c = in.read();
      if (c != -1)
        return (char)c;
      if (start == 0)
        start = System.currentTimeMillis();
      else
        now = System.currentTimeMillis();

    } while (now < (start + timeout));
 
    throw new Exception("timeout getting data from aGarciaAgent");
  }


  /////////////////////////////////////////////////////////////////

  public void write (
    DataOutputStream out,
    Object data
  ) throws Exception
  {
    int type = nType & aPROPERTY_TYPE_MASK;

    switch (type) {

    case aPROPERTY_FLAG_CALLBACK:
      {
        Integer i = (Integer)data;
        out.writeInt(i.intValue());
      }
      break;

    case aPROPERTY_FLAG_FLOAT:
      {
        Float f = (Float)data;
        out.writeFloat(f.floatValue());
      }
      break;

    case aPROPERTY_FLAG_INT:
      {
        Integer i = (Integer)data;
        out.writeInt(i.intValue());
      }
      break;

    case aPROPERTY_FLAG_STRING:
      {
        String s = (String)data;
        out.writeBytes(s);
        out.write(0);
      }
      break;

    } // switch

  } // write method



  /////////////////////////////////////////////////////////////////

  public Object read (
    DataInputStream in
  ) throws Exception
  {
    int type = nType & aPROPERTY_TYPE_MASK;
  
    switch (type) {

    case aPROPERTY_FLAG_INT:
      {
        int val;
        int i;

        i = nextByte(in);
        val = (i << 24);
        i = nextByte(in);
        val += (i << 16);
        i = nextByte(in);
        val += (i << 8);
        i = nextByte(in);
        val += i;
        return new Integer(val);
      }

    case aPROPERTY_FLAG_BOOL:
      {
        int b = nextByte(in);
        return new Boolean(b == 1);
      }

    case aPROPERTY_FLAG_FLOAT:
      {
        float f = in.readFloat();
        return new Float(f);
      }

    } // switch 
    
    return null;

  } // read method



  /////////////////////////////////////////////////////////////////

  public void send (
    jObject object,
    Object value,
    DataOutputStream out
  ) throws Exception
  {
    out.writeByte('W');
    out.writeInt(object.getAddr());
    out.writeShort(nIndex);
    out.writeInt(nType & aPROPERTY_TYPE_MASK);    
    write(out, value);
    out.flush();
  }


  /////////////////////////////////////////////////////////////////

  public void request (
    jObject object,
    DataOutputStream out
  ) throws Exception
  {
    out.writeByte('R');
    out.writeInt(object.getAddr());
    out.writeShort(nIndex);
    out.writeInt(nType & aPROPERTY_TYPE_MASK);
    out.flush();
  }

} // jProperty class
