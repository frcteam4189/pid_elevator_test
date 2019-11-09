/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDriveCommand;
//import edu.wpi.first.wpilibj.PIDOutput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.kauailabs.navx.frc.AHRS;


/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  DigitalInput limitSwitch = new DigitalInput(RobotMap.limitSwitchPort);

  //public static Encoder encoder = new Encoder(RobotMap.encA1Port, RobotMap.encA2Port, false, Encoder.EncodingType.k4X);

  DigitalInput encA1 = new DigitalInput(RobotMap.encA1Port);
	DigitalInput encA2 = new DigitalInput(RobotMap.encA2Port);
	public Encoder encA = new Encoder(encA1, encA2);
  // public Talon leftMaster = new Talon(RobotMap.leftMasterPort);
  // public Talon leftSlave = new Talon(RobotMap.leftSlavePort);
  // public Talon rightMaster = new Talon(RobotMap.rightMasterPort);
  // public Talon rightSlave = new Talon(RobotMap.rightSlavePort);
  public Talon elevator = new Talon(RobotMap.elevatorMotorPort);
  //public double angle = OI.ahrs.getAngle();
  

  // Solenoid p1 = new Solenoid(RobotMap.pneumaticPort1);
	// Solenoid p2 = new Solenoid(RobotMap.pneumaticPort2);
	// public Compressor compressor = new Compressor(0);


  public WPI_TalonSRX leftFront = new WPI_TalonSRX(0);
  public WPI_TalonSRX leftBack = new WPI_TalonSRX(3);
  public WPI_TalonSRX rightFront = new WPI_TalonSRX(2);
  public WPI_TalonSRX rightBack = new WPI_TalonSRX(1);
  // SpeedControllerGroup motorGroup = new SpeedControllerGroup(leftMaster, leftSlave, rightMaster, rightSlave);

  //public MecanumDrive drive = new MecanumDrive(leftMaster, leftSlave, rightMaster, rightSlave);

  public MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  public double ultraSonicFrontGet(){
    return OI.ultraSonicFront.getVoltage();
  }
  // public void openSet(boolean x){
  //   Robot.driveSubsystem.p1.set(x);
  //   Robot.driveSubsystem.p2.set(x);
	// }
	// public void closeSet(boolean x){
  //   Robot.driveSubsystem.p1.set(x);
  //   Robot.driveSubsystem.p2.set(x);
  // }
  public void elevatorSet(double x){
		Robot.driveSubsystem.elevator.set(x);
	}
  public double encAGetDistance()  {
    return encA.getDistance();
  }
  public double encAGet() {
    return encA.get()/100;
  }
  public double encAGetRate() {
    return encA.get();
  }
  public boolean limitSwitchGet() {
    return limitSwitch.get();
  }
  public double opticDeviceGet() {
    return OI.opticDevice.getVoltage();
  }
  public void manualDrive(double move, double translate, double turn){

    // if(move >= .7) move = .7;
    // if(move <= -.7) move = -.7;
    // if(translate >= .7) translate = .7;
    // if(translate <= -.7) translate = -.7;
    // if(turn >= .7) turn = .7;
    // if(turn <= -.7) turn = -.7;

    // if(move <= .2 && move > 0) move = 0;
    // if(move >= -.2 && move < 0) move = 0;
    // if(translate <= .2 && translate > 0) translate = 0;
    // if(translate >= -.2 && translate < 0) translate = 0;
    // if(turn <= .2 && turn > 0) turn = 0;
    // if(turn >= -.2 && turn < 0) turn = 0;

    drive.driveCartesian(move, translate, turn);
    //leftFront.setInverted(true);
    //rightBack.setInverted(true);
    //rightFront.setInverted(true);
    //leftBack.setInverted(true);
  
  }

  public void dashData(){
    SmartDashboard.putNumber("UltraSonicFront: ", Robot.driveSubsystem.ultraSonicFrontGet());
    SmartDashboard.putBoolean("Limit Switch", limitSwitchGet());
    SmartDashboard.putNumber("Optic Device", Robot.driveSubsystem.opticDeviceGet());
    SmartDashboard.putNumber("Encoder Distance", Robot.driveSubsystem.encAGetDistance());
    SmartDashboard.putNumber("Encoder Rate", Robot.driveSubsystem.encAGetRate());
    SmartDashboard.putNumber("Encoder Get", Robot.driveSubsystem.encAGet());
    SmartDashboard.putNumber("Gyro Angle", OI.ahrs.getAngle());
    // SmartDashboard.putNumber("Compressor", Robot.driveSubsystem.compressor.getCompressorCurrent());
    // SmartDashboard.putBoolean("PSV", Robot.driveSubsystem.compressor.getPressureSwitchValue());
  
    /* Display 6-axis Processed Angle Data                                      */
    // SmartDashboard.putBoolean(  "IMU_Connected", OI.ahrs.isConnected());
    // SmartDashboard.putBoolean(  "IMU_IsCalibrating", OI.ahrs.isCalibrating());
    // SmartDashboard.putNumber(   "IMU_Yaw", OI.ahrs.getYaw());
    // SmartDashboard.putNumber(   "IMU_Pitch", OI.ahrs.getPitch());
    // SmartDashboard.putNumber(   "IMU_Roll", OI.ahrs.getRoll());
    
    // /* Display tilt-corrected, Magnetometer-based heading (requires             */
    // /* magnetometer calibration to be useful)                                   */
    
    // SmartDashboard.putNumber(   "IMU_CompassHeading", OI.ahrs.getCompassHeading());
    
    // /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    // SmartDashboard.putNumber(   "IMU_FusedHeading", OI.ahrs.getFusedHeading());

    // /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    // /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
    
    // SmartDashboard.putNumber(   "IMU_TotalYaw", OI.ahrs.getAngle());
    // SmartDashboard.putNumber(   "IMU_YawRateDPS", OI.ahrs.getRate());

    // /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    
    // SmartDashboard.putNumber(   "IMU_Accel_X", OI.ahrs.getWorldLinearAccelX());
    // SmartDashboard.putNumber(   "IMU_Accel_Y", OI.ahrs.getWorldLinearAccelY());
    // SmartDashboard.putBoolean(  "IMU_IsMoving", OI.ahrs.isMoving());
    // SmartDashboard.putBoolean(  "IMU_IsRotating", OI.ahrs.isRotating());

    // /* Display estimates of velocity/displacement.  Note that these values are  */
    // /* not expected to be accurate enough for estimating robot position on a    */
    // /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    // /* of these errors due to single (velocity) integration and especially      */
    // /* double (displacement) integration.                                       */
    
    // SmartDashboard.putNumber(   "Velocity_X", OI.ahrs.getVelocityX());
    // SmartDashboard.putNumber(   "Velocity_Y", OI.ahrs.getVelocityY());
    // SmartDashboard.putNumber(   "Displacement_X", OI.ahrs.getDisplacementX());
    // SmartDashboard.putNumber(   "Displacement_Y", OI.ahrs.getDisplacementY());
    
    // /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
    // /* NOTE:  These values are not normally necessary, but are made available   */
    // /* for advanced users.  Before using this data, please consider whether     */
    // /* the processed data (see above) will suit your needs.                     */
    
    // SmartDashboard.putNumber(   "RawGyro_X", OI.ahrs.getRawGyroX());
    // SmartDashboard.putNumber(   "RawGyro_Y", OI.ahrs.getRawGyroY());
    SmartDashboard.putNumber(   "RawGyro_Z", OI.ahrs.getRawGyroZ());
    // SmartDashboard.putNumber(   "RawAccel_X", OI.ahrs.getRawAccelX());
    // SmartDashboard.putNumber(   "RawAccel_Y", OI.ahrs.getRawAccelY());
    // SmartDashboard.putNumber(   "RawAccel_Z", OI.ahrs.getRawAccelZ());
    // SmartDashboard.putNumber(   "RawMag_X", OI.ahrs.getRawMagX());
    // SmartDashboard.putNumber(   "RawMag_Y", OI.ahrs.getRawMagY());
    // SmartDashboard.putNumber(   "RawMag_Z", OI.ahrs.getRawMagZ());
    // SmartDashboard.putNumber(   "IMU_Temp_C", OI.ahrs.getTempC());
    
    // /* Omnimount Yaw Axis Information                                           */
    // /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    // AHRS.BoardYawAxis yaw_axis = OI.ahrs.getBoardYawAxis();
    // SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
    // SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
    
    // /* Sensor Board Information                                                 */
    // SmartDashboard.putString(   "FirmwareVersion", OI.ahrs.getFirmwareVersion());
    
    // /* Quaternion Data                                                          */
    // /* Quaternions are fascinating, and are the most compact representation of  */
    // /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    // /* from the Quaternions.  If interested in motion processing, knowledge of  */
    // /* Quaternions is highly recommended.                                       */
    // SmartDashboard.putNumber(   "QuaternionW", OI.ahrs.getQuaternionW());
    // SmartDashboard.putNumber(   "QuaternionX", OI.ahrs.getQuaternionX());
    // SmartDashboard.putNumber(   "QuaternionY", OI.ahrs.getQuaternionY());
    // SmartDashboard.putNumber(   "QuaternionZ", OI.ahrs.getQuaternionZ());
    
    // /* Connectivity Debugging Support                                           */
    // SmartDashboard.putNumber(   "IMU_Byte_Count", OI.ahrs.getByteCount());
    // SmartDashboard.putNumber(   "IMU_Update_Count", OI.ahrs.getUpdateCount());
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualDriveCommand());
  }
}