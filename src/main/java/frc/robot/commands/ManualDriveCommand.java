/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class ManualDriveCommand extends Command {
  public double distance;
  public ManualDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  //   if(OI.openPneumatics.get()){
  //     Robot.driveSubsystem.openSet(true);
  //     Timer.delay(2);
  //     Robot.driveSubsystem.openSet(false);	   
  //     }
  //  if(OI.closePneumatics.get()){
  //       Robot.driveSubsystem.openSet(false);	   
  //  }
   
    // if(OI.upElevatorH1.get()) {
    //   Robot.driveSubsystem.elevatorSet(1);
    //   if(Robot.driveSubsystem.encAGet() == 10) {
    //     Robot.driveSubsystem.elevator.set(0);
    //   }
    //   else if(Robot.driveSubsystem.encAGet() > 10) {
    //     Robot.driveSubsystem.elevator.set(-1);
    //   }
    // }
    // else if(OI.upElevatorH2.get())  {
    //   Robot.driveSubsystem.elevatorSet(1);
    //   if(Robot.driveSubsystem.encAGet() == 20) {
    //     Robot.driveSubsystem.elevator.set(0);
    //   }
    //   else if(Robot.driveSubsystem.encAGet() > 20) {
    //     Robot.driveSubsystem.elevator.set(-1);
    //   }
    // }
    // else if(OI.upElevatorH3.get())  {
    //   Robot.driveSubsystem.elevatorSet(1);
    //   if(Robot.driveSubsystem.encAGet() == 30) {
    //     Robot.driveSubsystem.elevator.set(0);
    //   }
    //   else if(Robot.driveSubsystem.encAGet() > 30) {
    //     Robot.driveSubsystem.elevator.set(-1);
    //   }
    // }
    // else if(OI.elevatorReset.get()) {
    //   Robot.driveSubsystem.elevatorSet(1);
    //   if(Robot.driveSubsystem.encAGetDistance() > 0)  {
    //     Robot.driveSubsystem.elevator.set(-1);
    //   }
    //   else if(Robot.driveSubsystem.encAGetDistance() <= 50 && Robot.driveSubsystem.encAGetDistance() >= -50) {
    //     Robot.driveSubsystem.elevator.set(0);
    //   }
    // }
    // else  {
    //   Robot.driveSubsystem.elevatorSet(0);
    // }

    // if(OI.servoMoveUp.get())  {
    //   distance = distance+0.005;
    //   OI.servoY.set(distance);
    // }
    // else if (OI.servoMoveDown.get())  {
    //   distance = distance-0.005;
    //   OI.servoY.set(distance);
    // }
    // else if (OI.servoMoveLeft.get())  {
    //   distance = distance-0.005;
    //   OI.servoX.set(distance);
    // }
    // else if (OI.servoMoveRight.get())  {
    //   distance = distance+0.005;
    //   OI.servoX.set(distance);
    // }

    Robot.driveSubsystem.manualDrive(OI.stick.getX(), OI.stick.getY()*-1, OI.stick.getZ());
    //Robot.driveSubsystem.manualDrive(0, 0, 1, 0.0);
    // Robot.driveSubsystem.manualDrive(-OI.ahrs.getAngle()*.003, OI.stick.getY(), OI.stick.getX()); // drive towards heading 0
    // Timer.delay(0.004);
    // double move = Robot.oi.stick.getY();
    // double translate = Robot.oi.stick.getX();
    // double turn = -Robot.oi.stick.getZ();
    // Robot.driveSubsystem.manualDrive(move, translate, turn);

    // if(OI.limitSwitch.get() == false)  {
    //   SpeedControllerGroup rightMotors = new SpeedControllerGroup(Robot.driveSubsystem.rightMaster, Robot.driveSubsystem.rightSlave);
    //   rightMotors.set(0);
    //   SpeedControllerGroup leftMotors = new SpeedControllerGroup(Robot.driveSubsystem.leftMaster, Robot.driveSubsystem.leftSlave);
    //   leftMotors.setInverted(true);
    //   leftMotors.set(0);
    // }

    // if (OI.opticDevice.getVoltage() <= 2)  {
    //   SpeedControllerGroup rightMotors = new SpeedControllerGroup(Robot.driveSubsystem.rightMaster, Robot.driveSubsystem.rightSlave);
    //   rightMotors.set(0);
    //   SpeedControllerGroup leftMotors = new SpeedControllerGroup(Robot.driveSubsystem.leftMaster, Robot.driveSubsystem.leftSlave);
    //   leftMotors.setInverted(true);
    //   leftMotors.set(0);
    // }

    // if(Robot.driveSubsystem.encA.get() >= )  {
    //   Robot.driveSubsystem.elevator.set(0);
    // }
    
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
