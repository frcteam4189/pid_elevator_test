/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.DriveStraight;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //USB
  public static Joystick stick = new Joystick(RobotMap.joyStickPort);
  

  // public static Button elevatorReset = new JoystickButton(stick, 4);
  public static Button pidElevator = new JoystickButton(stick, 11);
  
  public static Button openPneumatics = new JoystickButton(stick, 3);
  public static Button closePneumatics = new JoystickButton(stick, 2);

  public static VideoCamera camera;
  //Joystick Buttons
   //PWM
  //public static Servo servoY = new Servo(RobotMap.servoYPort);
  //public static Servo servoX = new Servo(RobotMap.servoXPort);
  
  //Analog
  public static AnalogInput ultraSonicFront = new AnalogInput(RobotMap.ultraSonicFrontPort);
  //public static AnalogInput ultraSonicBack = new AnalogInput(RobotMap.ultraSonicBackPort);
  public static AnalogInput opticDevice = new AnalogInput(RobotMap.opticDevicePort);
  public static AHRS ahrs = new AHRS(SPI.Port.kMXP);
  
  public static Button driveStraight = new JoystickButton(stick, 1);
  public OI(){
  driveStraight.toggleWhenPressed(new DriveStraight());
  }
}