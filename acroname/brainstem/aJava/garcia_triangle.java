/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: garcia_triangle.java					   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Simple example program that shows the use of 	   //
//		Java with the Garcia robot.  In this example,	   //
//              the robot is driven in a triangle.		   //
//                                                                 //
//              You should be able to build this example after     //
//		making changes using the javac command:            //
//		                                                   //
//			javac garcia_triangle.java		   //
//		                                                   //
//              You should be able to run this example after with  //
//		java command:  				           //
//		                                                   //
//			java garcia_triangle			   //
//		                                                   //
//              and you may need to preceed the command with the   //
//		proper path to the java distribution that has      //
//		the comm package installed, for example:           //
//		                                                   //
//			C:\j2sdk1.4.0\jre\bin\java gp_example	   //
//		                                                   //
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

import acroname.*;



/////////////////////////////////////////////////////////////////////

class executeCallback
  extends jCallback
{
  public void call()
  {
    try {
      System.out.println("behavior " 
    			+ (String)(behavior.getValue("name"))
    			+ " with id "
    			+ (Integer)(behavior.getValue("unique-id"))
    			+ " executing...");
    } catch (Exception e) {
      System.out.println(e.toString());
    }
  }
}



/////////////////////////////////////////////////////////////////////

class completionCallback
  extends jCallback
{
  public void call()
  {
    try {
      System.out.println("behavior " 
    			+ (String)(behavior.getValue("name"))
    			+ " exited with status "
    			+ (Integer)(behavior.getValue("completion-status"))
    			+ ".");
    } catch (Exception e) {
      System.out.println(e.toString());
    }
  }
}



/////////////////////////////////////////////////////////////////////

public class garcia_triangle
{

  /////////////////////////////////////////////////////////////////
  // main
  //
  public static void main(String[] args)
  {
    jRobot robot = null;
    jBehavior behavior = null;
    Thread current = Thread.currentThread();

    try {
      robot = new jRobot();

      System.out.println("waiting for robot");
      
      while (!((Boolean)robot.getValue("active")).booleanValue()) {
        System.out.println("still waiting");
        current.sleep(100);
      }

      System.out.println("queueing a simple triangle");

      Object fLength = new Float(0.5f);
      Object fRotation = new Float(((float)(3.1459 * 2 / 3)));
      behavior = robot.createBehavior("move", "out");
      behavior.setValue("distance", fLength);
      behavior.setValue("execute-callback", 
      		    new executeCallback());
      behavior.setValue("completion-callback", 
      		    new completionCallback());
      robot.queueBehavior(behavior);

      behavior = robot.createBehavior("pivot", "pivot 1");
      behavior.setValue("angle", fRotation);
      behavior.setValue("execute-callback", 
      		    new executeCallback());
      behavior.setValue("completion-callback", 
      		    new completionCallback());
      robot.queueBehavior(behavior);

      behavior = robot.createBehavior("move", "over");
      behavior.setValue("distance", fLength);
      behavior.setValue("execute-callback", 
      		    new executeCallback());
      behavior.setValue("completion-callback", 
      		    new completionCallback());
      robot.queueBehavior(behavior);

      behavior = robot.createBehavior("pivot", "pivot 2");
      behavior.setValue("angle", fRotation);
      behavior.setValue("execute-callback", 
      		    new executeCallback());
      behavior.setValue("completion-callback", 
      		    new completionCallback());
      robot.queueBehavior(behavior);

      behavior = robot.createBehavior("move", "back");
      behavior.setValue("distance", fLength);
      behavior.setValue("execute-callback", 
      		    new executeCallback());
      behavior.setValue("completion-callback", 
      		    new completionCallback());
      robot.queueBehavior(behavior);

      behavior = robot.createBehavior("pivot", "pivot 3");
      behavior.setValue("angle", fRotation);
      behavior.setValue("execute-callback", 
      		    new executeCallback());
      behavior.setValue("completion-callback", 
      		    new completionCallback());
      robot.queueBehavior(behavior);
      
      // this loop checks to see when the list above is complete
      // and checks for callbacks in 100 mSec intervals
      System.out.println("waiting till idle");
      while (!((Boolean)robot.getValue("idle")).booleanValue())
        robot.handleCallbacks(100);

    }
    catch (Exception e) {
      System.out.println("error: " + e.toString());
      e.printStackTrace();
    }

    System.out.println("done");

  } // main

} // garcia_triangle class
