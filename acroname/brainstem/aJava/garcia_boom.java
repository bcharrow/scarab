/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: garcia_boom.java                                          //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Simple example program that shows the use of 	   //
//		Java with the Garcia robot.  In this example,	   //
//              the robot wiggles the camera boom pana and tilt.   //
//                                                                 //
//              You should be able to build this example after     //
//		making changes using the javac command:            //
//		                                                   //
//			javac garcia_boom.java			   //
//		                                                   //
//              You should be able to run this example after with  //
//		java command:  				           //
//		                                                   //
//			java garcia_boom			   //
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

public class garcia_boom
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

      System.out.println("moving boom");

      behavior = robot.createBehavior("null", "wait");

      // The "null" primitive requires an acceleration 
      // property even though we are not moving the 
      // robot in this example
      behavior.setValue("acceleration", new Float(0.5f));
      robot.queueBehavior(behavior);

      // this loop checks to see when the list above is 
      // complete and checks for callbacks in 100 mSec 
      // intervals 
      float diff = 0.08f;
      float inc = diff;
      float val = 0.0f;
      while (!((Boolean)robot.getValue("idle")).booleanValue()) {

	// get the camera object by classname and name
        jObject camera = robot.getSubObject("camera", 
        				     "camera-boom");

        // set some properties on the boom to move it
        camera.setValue("pan", new Float(val));
        camera.setValue("tilt", new Float(val));

	// adjust for the next pass
        val += inc;
        if (val >= 0.5)
          inc = -diff;
        if (val <= -0.5)
          inc = diff;

	// allow some system processing
        robot.handleCallbacks(20);
      }

      // catch any final callbacks
      robot.handleCallbacks(0);
    }
    catch (Exception e) {
      System.out.println("error: " + e.toString());
      e.printStackTrace();
    }

    robot.finalize();
    System.out.println("done");

  } // main

} // garcia_boom class
