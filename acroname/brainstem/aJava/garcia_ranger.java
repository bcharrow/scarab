/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: garcia_ranger.java                                        //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Simple example program that shows the use of 	   //
//		Java with the Garcia robot.  In this example,	   //
//              the robot moves forward to an obstruction, stops.  //
//              and then reports the distance to both front        //
//              rangers.                                           //
//                                                                 //
//              You should be able to build this example after     //
//		making changes using the javac command:            //
//		                                                   //
//			javac garcia_ranger.java		   //
//		                                                   //
//              You should be able to run this example after with  //
//		java command:  				           //
//		                                                   //
//			java garcia_ranger			   //
//		                                                   //
//              and you may need to preceed the command with the   //
//		proper path to the java distribution that has      //
//		the comm package installed, for example:           //
//		                                                   //
//			C:\j2sdk1.4.0\jre\bin\java java_ranger	   //
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
// This class offers the callback hook to catch the null behavior's
// exit.  An object of this class is set on the behavior object
// that gets called when the behavior completes.

class rangerCompletionCallback
  extends jCallback
{
  public void call()
  {
    try {
      // Get the completion status which will likely be an
      // IR input since we don't have anything else happening.
      // One other candidate error would be a downward-looking
      // IR sensor if you picked up the robot.
      int status = 
    	((Integer)behavior.getValue("completion-status")).intValue();
      System.out.println("behavior " 
    			+ (String)(behavior.getValue("name"))
    			+ " exited with status "
    			+ status
    			+ ".");
    } catch (Exception e) {
      System.out.println("couldn't get completion status: " 
      			 + e.toString());
    } // catch
  } // call method
} // rangerCompletionCallback



/////////////////////////////////////////////////////////////////////

public class garcia_ranger
{

  /////////////////////////////////////////////////////////////////
  // main
  //
  public static void main(String[] args)
  {
    jGarcia garcia = null;
    jBehavior behavior = null;
    Thread current = Thread.currentThread();

    try {
      garcia = new jGarcia();

      System.out.println("waiting for garcia");

      // block until garcia is ready to go
      while (!((Boolean)garcia.getValue("active")).booleanValue()) {
        System.out.println("still waiting");
        current.sleep(100);
      }

/*
      // Configure the IR input to terminate a primitive in
      // progress.
      System.out.println("configuring for IR input");
      int flags = ((Integer)garcia.getValue("user-flags")).intValue();
      flags |= garcia.aGARCIA_EXECTRL_IRCHK;
      garcia.setValue("user-flags", new Integer(flags));

      // Now, create a null behavior where we will wait
      // for IR input.
      System.out.println("waiting for ir input");
*/

      behavior = garcia.createBehavior("null", "wait");
      behavior.setValue("completion-callback", 
      		        new rangerCompletionCallback());

      // The "null" primitive requires an acceleration 
      // property
      behavior.setValue("acceleration", new Float(0.5f));
      garcia.queueBehavior(behavior);

      // start moving forward
      garcia.setValue("damped-speed-left", new Float(0.2f));
      garcia.setValue("damped-speed-right", new Float(0.2f));

      // this loop checks to see when the list above is complete
      // and checks for callbacks in 100 mSec intervals  
      while (!((Boolean)garcia.getValue("idle")).booleanValue())
        garcia.handleCallbacks(100);
      
      // now, we just get the ir range values
      Float left = ((Float)garcia.getValue("front-ranger-left"));
      Float right = ((Float)garcia.getValue("front-ranger-right"));
      System.out.println("left = " 
      			 + left.floatValue() 
      			 + " right = " 
      			 + right.floatValue());

      left = ((Float)garcia.getValue("front-ranger-left"));
      right = ((Float)garcia.getValue("front-ranger-right"));
      System.out.println("left = " 
      			 + left.floatValue() 
      			 + " right = " 
      			 + right.floatValue());

      left = ((Float)garcia.getValue("front-ranger-left"));
      right = ((Float)garcia.getValue("front-ranger-right"));
      System.out.println("left = " 
      			 + left.floatValue() 
      			 + " right = " 
      			 + right.floatValue());

      left = ((Float)garcia.getValue("front-ranger-left"));
      right = ((Float)garcia.getValue("front-ranger-right"));
      System.out.println("left = " 
      			 + left.floatValue() 
      			 + " right = " 
      			 + right.floatValue());

      // we need a final yield here to allow any final processing
      // to transpire
      garcia.handleCallbacks(50);
    }
    catch (Exception e) {
      System.out.println("error: " + e.toString());
      e.printStackTrace();
    }

    garcia.finalize();
    System.out.println("done");

  } // main

} // garcia_ir class
