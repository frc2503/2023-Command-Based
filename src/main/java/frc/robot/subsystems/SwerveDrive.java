// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
Class to hold all code for the swerve drive
Mostly exists to reduce verbosity in Robot.java, but it also helps prevent people from accidentally messing the code up
*/
public class SwerveDrive extends SubsystemBase {
  // Define all objects and varibles used by this class
  public AHRS Gyro;
  public Rotation2d GyroRotation2d;

  public SwerveDriveKinematics Kinematics;
  public SwerveDriveOdometry Odometry;

  private ChassisSpeeds Speeds;
  private SwerveModuleState[] ModuleStates;
  public SwerveModulePosition[] ModulePositions;
  
  public Wheel FrontRight;
  public Wheel FrontLeft;
  public Wheel BackLeft;
  public Wheel BackRight;

  public boolean FieldOrientedSwerveEnabled;

  /**
   * SwerveDrive class constructor, initializes all variables, objects, and methods for the created SwerveDrive object
   */
  public SwerveDrive() {
    // Create objects for the Wheel class, and define locations of wheel modules compared to robot center, which doesn't really matter unless base is MUCH longer on one side
    FrontRight = new Wheel(0.2604, -0.2786);
    FrontLeft = new Wheel(0.2604, 0.2786);
    BackLeft = new Wheel(-0.2604, -0.2786);
    BackRight = new Wheel(-0.2604, 0.2786);

    // Create a ChassisSpeeds object, which we later pass our desired speeds into to get our wheel speeds and angles
    Speeds = new ChassisSpeeds();
    
    // Initialize and zero gyro
    Gyro = new AHRS(SerialPort.Port.kMXP);
    Gyro.calibrate();
    Gyro.reset();
    GyroRotation2d = Gyro.getRotation2d();

    FrontRight.Drive = new CANSparkMax(Constants.FrontRightDriveCANID, MotorType.kBrushless);
    FrontRight.Steer = new TalonSRX(Constants.FrontRightSteerCANID);
    FrontLeft.Drive = new CANSparkMax(Constants.FrontLeftDriveCANID, MotorType.kBrushless);
    FrontLeft.Steer = new TalonSRX(Constants.FrontLeftSteerCANID);
    BackRight.Drive = new CANSparkMax(Constants.BackRightDriveCANID, MotorType.kBrushless);
    BackRight.Steer = new TalonSRX(Constants.BackRightSteerCANID);
    BackLeft.Drive = new CANSparkMax(Constants.BackLeftDriveCANID, MotorType.kBrushless);
    BackLeft.Steer = new TalonSRX(Constants.BackLeftSteerCANID);
    
    FrontRight.initEncodersAndPIDControllers();
    FrontLeft.initEncodersAndPIDControllers();
    BackLeft.initEncodersAndPIDControllers();
    BackRight.initEncodersAndPIDControllers();

    updatePID(Constants.DriveFeedForward, Constants.DriveProportional, Constants.DriveIntegral, Constants.DriveDerivative, Constants.SteerFeedForward, Constants.SteerProportional, Constants.SteerIntegral, Constants.SteerDerivative);
    initKinematicsAndOdometry();

    // We usually want this enabled, but toggling this off will treat the camera angle as "forward"
    FieldOrientedSwerveEnabled = true;
  }

  public void initKinematicsAndOdometry() {
    ModulePositions = new SwerveModulePosition[] {FrontRight.getPosition(), FrontLeft.getPosition(), BackLeft.getPosition(), BackRight.getPosition()};
    
    // Pass in locations of wheels relative to the center of the robot
    // These are later used in the backend, likely to find the angles the wheels need to rotate to when the robot spins 
    Kinematics = new SwerveDriveKinematics(FrontRight.Location, FrontLeft.Location, BackLeft.Location, BackRight.Location);
    
    // Pass in wheel module locations, as well as initial robot angle and position for field oriented drive
    Odometry = new SwerveDriveOdometry(Kinematics, GyroRotation2d, ModulePositions, new Pose2d(0, 0, new Rotation2d()));
  }
  /**
	 * Assign motor controllers their CAN numbers, and call the initEncodersAndPIDControllers() method for each wheel module
	 *
	 * @param FRD
	 *            CAN number of the front right drive motor controller
	 * @param FRS
	 *            CAN number of the front right steer motor controller
   * @param FLD
	 *            CAN number of the front left drive motor controller
   * @param FLS
	 *            CAN number of the front left steer motor controller
   * @param BLD
	 *            CAN number of the back left drive motor controller
   * @param BLS
	 *            CAN number of the back left steer motor controller
   * @param BRD
	 *            CAN number of the back right drive motor controller
   * @param BRS
	 *            CAN number of the back right steer motor controller
	 */
  public void initMotorControllers(Integer FRD, Integer FRS, Integer FLD, Integer FLS, Integer BLD, Integer BLS, Integer BRD, Integer BRS) {
    FrontRight.Drive = new CANSparkMax(FRD, MotorType.kBrushless);
    FrontRight.Steer = new TalonSRX(FRS);
    FrontLeft.Drive = new CANSparkMax(FLD, MotorType.kBrushless);
    FrontLeft.Steer = new TalonSRX(FLS);
    BackRight.Drive = new CANSparkMax(BRD, MotorType.kBrushless);
    BackRight.Steer = new TalonSRX(BRS);
    BackLeft.Drive = new CANSparkMax(BLD, MotorType.kBrushless);
    BackLeft.Steer = new TalonSRX(BLS);
    
    FrontRight.initEncodersAndPIDControllers();
    FrontLeft.initEncodersAndPIDControllers();
    BackLeft.initEncodersAndPIDControllers();
    BackRight.initEncodersAndPIDControllers();
  }

  /**
   * Call the setPIDValues() method for each wheel module
   * 
   * @param DFF
   *            Drive Feed-Forward value
   * @param DP
	 *            Drive Proportional value
   * @param DI
	 *            Drive Integral value
   * @param DD
	 *            Drive Derivative value
   * @param SFF
	 *            Steer Feed Forward value
   * @param SP
	 *            Steer Proportional value
   * @param SI
	 *            Steer Integral value
   * @param SD
	 *            Steer Derivative value
   */
  public void updatePID(double DFF, double DP, double DI, double DD, double SFF, double SP, double SI, double SD) {
    FrontRight.setPIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
    FrontLeft.setPIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
    BackLeft.setPIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
    BackRight.setPIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
  }

  /**
   * Do all of the math to calculate the speeds and angles for each wheel on the swerve drive
   * 
   * @param X
	 *            Desired X speed of the robot from -1 to 1
   * @param Y
	 *            Desired Y speed of the robot from -1 to 1
   * @param Spin
	 *            Desired rotational speed of the robot from -1 to 1
   * @param XYMod
   *            Number to multiply the translation speed of the robot by, to modify speed while driving
   * @param SpinMod
   *            Number to multiply the rotation speed of the robot by, to modify speed while driving
   */
  public void calculateSpeedsAndAngles(double X, double Y, double Spin, double XYMod, double SpinMod) {
    // Set the desired speeds for the robot, we also pass in the gyro angle for field oriented drive
    if (FieldOrientedSwerveEnabled) {
      Speeds = ChassisSpeeds.fromFieldRelativeSpeeds((Y * XYMod), (X * XYMod), (Spin * SpinMod), GyroRotation2d);
    } else {
      Speeds = ChassisSpeeds.fromFieldRelativeSpeeds((Y * XYMod), (X * XYMod), (Spin * SpinMod), new Rotation2d(0));
    }

    // Convert overall robot speeds and angle into speeds and angles for each wheel module, referred to as module states
    ModuleStates = Kinematics.toSwerveModuleStates(Speeds);

    // Front left module state
    FrontLeft.ModuleState = ModuleStates[0];

    // Front right module state
    FrontRight.ModuleState = ModuleStates[1];

    // Back left module state
    BackLeft.ModuleState = ModuleStates[2];

    // Back right module state
    BackRight.ModuleState = ModuleStates[3];
  }

  /**
   * Do all of the math to optimize wheel angles, and output to the wheel modules
   */
  public void optimizeAndSetOutputs() {
    // Do math to get multiple variables out of the encoder position
    FrontLeft.setEncoderVariables();
    FrontRight.setEncoderVariables();
    BackLeft.setEncoderVariables();
    BackRight.setEncoderVariables();
    
    // Do math for swerve drive that is identical between all wheel modules, and then send the angle and speed to the wheels
    FrontRight.optimizeAndCalculateVariables();
    FrontLeft.optimizeAndCalculateVariables();
    BackLeft.optimizeAndCalculateVariables();
    BackRight.optimizeAndCalculateVariables();

    // Update Odometry, so the robot knows its position on the field
    ModulePositions = new SwerveModulePosition[] {FrontRight.getPosition(), FrontLeft.getPosition(), BackLeft.getPosition(), BackRight.getPosition()};
    
    if (FieldOrientedSwerveEnabled) {
      Odometry.update(GyroRotation2d, ModulePositions);
    }
    else if (!FieldOrientedSwerveEnabled) {
      Odometry.update(new Rotation2d(0), ModulePositions);
    }

    FrontRight.setOutputs();
    FrontLeft.setOutputs();
    BackLeft.setOutputs();
    BackRight.setOutputs();
  }

  /**
   * Get the robot pose from the Odometry
   * 
   * @return The Pose2d of the robot
   */
  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }

  /**
   * Get the robot angle from the Odometry
   * 
   * @return The Rotation2d of the robot
   */
  public Rotation2d getRotation() {
    return GyroRotation2d;
  }

  public void setModuleStates(SwerveModuleState[] DesiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(DesiredStates, 2);
    ModuleStates = DesiredStates;
    
    // Front left module state
    FrontLeft.ModuleState = new SwerveModuleState(ModuleStates[0].speedMetersPerSecond, new Rotation2d((2 * Math.PI) - ModuleStates[0].angle.getRadians()));

    // Front right module state
    FrontRight.ModuleState = new SwerveModuleState(ModuleStates[1].speedMetersPerSecond, new Rotation2d((2 * Math.PI) - ModuleStates[1].angle.getRadians()));

    // Back left module state
    BackLeft.ModuleState = new SwerveModuleState(ModuleStates[2].speedMetersPerSecond, new Rotation2d((2 * Math.PI) - ModuleStates[2].angle.getRadians()));

    // Back right module state
    BackRight.ModuleState = new SwerveModuleState(ModuleStates[3].speedMetersPerSecond, new Rotation2d((2 * Math.PI) - ModuleStates[3].angle.getRadians()));

    //System.out.println(Odometry.getPoseMeters().getX());

    optimizeAndSetOutputs();
  }

  public void stop() {
    System.out.println("End X:" + Odometry.getPoseMeters().getX());
    System.out.println("End Y:" + Odometry.getPoseMeters().getY());
    calculateSpeedsAndAngles(0.0, 0.0, 0.0, 1.0, 1.0);
    optimizeAndSetOutputs();
  }
}
