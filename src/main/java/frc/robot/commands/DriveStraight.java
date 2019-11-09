/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;


public class DriveStraight extends Command {
  public DriveStraight() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // OI.ahrs.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if(OI.ahrs.getAngle() >= 360)  {
    //   Robot.driveSubsystem.angle = 0.0;
    // }
    // else if(OI.ahrs.getAngle() <= -360) {
    //   Robot.driveSubsystem.angle = 0.0;
    // }
			if (Math.abs(OI.stick.getY()) > 0.15) {
				Robot.driveSubsystem.manualDrive(-OI.ahrs.getAngle()*.3, OI.stick.getY()*-1, 0);
       }
      if (Math.abs(OI.stick.getX()) > 0.15) {
				Robot.driveSubsystem.manualDrive(-OI.ahrs.getAngle()*.3, 0, OI.stick.getX());
      } 
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
