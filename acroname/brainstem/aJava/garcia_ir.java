/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: garcia_ir.java                                            //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Simple example program that shows the use of 	   //
//		Java with the Garcia robot.  In this example,	   //
//              the robot reports an IR input and exits.           //
//                                                                 //
//              You should be able to build this example after     //
//		making changes using the javac command:            //
//		                                                   //
//			javac garcia_ir.java			   //
//		                                                   //
//              You should be able to run this example after with  //
//		java command:  				           //
//		                                                   //
//			java garcia_ir				   //
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
// This class offers the callback hook to catch the null behavior's
// exit.  An object of this class is set on the behavior object
// that gets called when the behavior completes.

class irCompletionCallback
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

      jGarcia garcia = (jGarcia)robot;
      // Now, if it was an IR receive event, we just display it.
      // Here, other errors are not handled.
      if (status == garcia.aGARCIA_ERRFLAG_IRRX) {
        // The "ir-receive" property is always updated with 
        // new IR inputs.  Here, we retreive the last one which
        // terminated our NULL primitive that was running.
        System.out.println("IR input was: " +
      			   (Integer)(robot.getValue("ir-receive")));
      	
      	// Reset this value so the next primitive doesn't exit
      	// because the value is still hanging around.  Sort of 
      	// like priming the system to accept the next IR input.
      	robot.setValue("ir-receive", new Integer(0));

      } // if 
    } catch (Exception e) {
      System.out.println("couldn't get ir-receive: " + e.toString());
    } // catch
  } // call method
} // irCompletionCallback



/////////////////////////////////////////////////////////////////////

public class garcia_ir
{

  /////////////////////////////////////////////////////////////////
  // main
  //
  public static void main(String[] args)
  {
    jGarcia robot = null;
    jBehavior behavior = null;
    Thread current = Thread.currentThread();

    try {
      robot = new jGarcia();

      System.out.println("waiting for robot");

      // block until robot is ready to go
      while (!((Boolean)robot.getValue("active")).booleanValue()) {
        System.out.println("still waiting");
        current.sleep(100);
      }

      // Configure the IR input to terminate a primitive in
      // progress.
      System.out.println("configuring for IR input");
      int flags = ((Integer)robot.getValue("user-flags")).intValue();
      flags |= robot.aGARCIA_EXECTRL_IRCHK;
      robot.setValue("user-flags", new Integer(flags));

      // Now, create a null behavior where we will wait
      // for IR input.
      System.out.println("waiting for ir input");

      behavior = robot.createBehavior("null", "wait");
      behavior.setValue("completion-callback", 
      		        new irCompletionCallback());
      // The "null" primitive requires an acceleration 
      // property
      behavior.setValue("acceleration", new Float(0.5f));
      robot.queueBehavior(behavior);

      robot.setValue("damped-speed-left", new Float(0.2f));
      robot.setValue("damped-speed-right", new Float(0.2f));

      // this loop checks to see when the list above is complete
      // and checks for callbacks in 100 mSec intervals  
      while (!((Boolean)robot.getValue("idle")).booleanValue())
        robot.handleCallbacks(500);

      // we need a final yield here to allow the final set of 
      // the ir receive to happen before the connection to the 
      // agent is torn down.
      robot.handleCallbacks(100);
    }
    catch (Exception e) {
      System.out.println("error: " + e.toString());
      e.printStackTrace();
    }

    robot.finalize();
    System.out.println("done");

  } // main

} // garcia_ir class
