/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //PWM
  public static int leftMasterPort = 2;
  public static int leftSlavePort = 1;
  public static int rightMasterPort = 4;
  public static int rightSlavePort = 3;
  public static int elevatorMotorPort = 5;
  public static int servoYPort = 4;
  public static int servoXPort = 5;

  //USB 
  public static int joyStickPort = 0;

  //Analog
  public static int ultraSonicFrontPort = 0;
  //public static int ultraSonicBackPort = 1;

  //DIO
  public static int limitSwitchPort = 0;
  public static int opticDevicePort = 1;
  public static int encA1Port = 1;
  public static int encA2Port = 2;
  // public static int encB1Port = 4;
  // public static int encB2Port = 5;

  public static int pneumaticPort1 = 0;
  public static int pneumaticPort2 = 1;
	
}