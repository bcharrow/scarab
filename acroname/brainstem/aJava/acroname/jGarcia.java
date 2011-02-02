/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: jGarcia.java                                               //
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

public class jGarcia extends jRobot
{  
  // completion status flags
  public final short aGARCIA_ERRFLAG_NORMAL = 		0x0000;
  public final short aGARCIA_ERRFLAG_STALL = 		0x0001;
  public final short aGARCIA_ERRFLAG_FRONTR_LEFT = 	0x0002;
  public final short aGARCIA_ERRFLAG_FRONTR_RIGHT = 	0x0003;
  public final short aGARCIA_ERRFLAG_REARR_LEFT = 	0x0004;
  public final short aGARCIA_ERRFLAG_REARR_RIGHT = 	0x0005;
  public final short aGARCIA_ERRFLAG_SIDER_LEFT = 	0x0008;
  public final short aGARCIA_ERRFLAG_SIDER_RIGHT = 	0x0009;
  public final short aGARCIA_ERRFLAG_FALL_LEFT	= 	0x0010;
  public final short aGARCIA_ERRFLAG_FALL_RIGHT = 	0x0011;
  public final short aGARCIA_ERRFLAG_ABORT = 		0x0012;
  public final short aGARCIA_ERRFLAG_NOTEXECUTED = 	0x0013;
  public final short aGARCIA_ERRFLAG_WONTEXECUTE = 	0x0014;
  public final short aGARCIA_ERRFLAG_BATT = 		0x0020;
  public final short aGARCIA_ERRFLAG_IRRX = 		0x0040;

  // completion status flags
  public final int aGARCIA_EXECTRL_USER =		0x0001;
  public final int aGARCIA_EXECTRL_IRCHK =		0x0002;
  public final int aGARCIA_EXECTRL_EDGECHK =		0x0004;
  public final int aGARCIA_EXECTRL_STALLCHK =		0x0008;

  // motor designations
  public final int aGARCIA_MOTO_MOTOR_RIGHT =		0x0000;
  public final int aGARCIA_MOTO_MOTOR_LEFT =		0x0001;

  /////////////////////////////////////////////////////////////////
 
  public jGarcia() throws Exception
  {
    super();

  } // jGarcia constructor


} // jGarcia class
