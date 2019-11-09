/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.buttons.Button;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.PIDController;
// import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.OI;
//import frc.robot.OI;
import frc.robot.Robot;
//import frc.robot.commands.ManualDriveCommand;

/**
 * Add your docs here.
 */
public class Elevator extends PIDSubsystem {
  private static final double kP = .47;

  // integral speed constant
  private static final double kI = 0;

  // derivative speed constant
  private static final double kD = 0;

  

  /**
   * Add your docs here.
   */
  public Elevator() {
    // Insert a subsystem name and PID values here
    super("Elevator", kP, kI, kD);
    
    // if(OI.upElevatorH1PID.get())  {
    //   enable();
    // }

    setSetpoint(5);
    enable();
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
   
    // enable() - Enables the PID controller.
  }

  
  // private int levels(){
  //   int result = 0;
  //   if(OI.upElevatorH1PID.get()){
  //     result = 1;
  //   }
  //   else if(OI.upElevatorH2PID.get()){
  //     result = 2;
  //   }
  //   else if(OI.upElevatorH3PID.get()){
  //     result = 3;
  //   }
  //   return result;
  // }
  // private void zoom(int input){
  //   if(input == 1)  {
  //     setSetpoint(10);
  //   }
  //   else if(input == 2) {
  //     setSetpoint(15);
  //   }
  //   else if(input == 3) {
  //     setSetpoint(20);
  //   }
  // }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new ManualDriveCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return Robot.driveSubsystem.encA.get()/100;
    
  }


  // public class MyPidOutput implements PIDOutput {
  //   @Override
  //   public void pidWrite(double output) {
  //   Robot.driveSubsystem.elevator.set(output);
  //   }
  // }
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    if(OI.pidElevator.get())  {
      Robot.driveSubsystem.elevator.set(output);
    }
    //Robot.driveSubsystem.elevator.set(output);
  }
}
