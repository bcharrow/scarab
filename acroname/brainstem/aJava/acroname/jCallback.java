/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: jCallback.java 					   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Interface for Garcia callback mechanism. 	   //
//								   //
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

public class jCallback 
  extends Object
{
  private int nID;
  public jBehavior behavior;
  protected jRobot robot;


  /////////////////////////////////////////////////////////////////

  public jCallback()
  {
  }


  /////////////////////////////////////////////////////////////////

  public void finalize(
    int nID,
    jBehavior behavior,
    jRobot robot
  )
  {
    this.nID = nID;
    this.behavior = behavior;
    this.robot = robot;
  }


  /////////////////////////////////////////////////////////////////

  public void call()
  {
    // subclasses can do something here
  }

} // jCallback class
