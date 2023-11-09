// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervesubsystem;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
* Holds most code for the swerve drive.
* Mostly exists to reduce verbosity in Robot.java, but it also helps prevent people from accidentally messing the code up.
 */
public class SwerveDrive extends SubsystemBase{
  // Define all objects and varibles used by this class
  public static AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  public static Rotation2d gyroRotation2d;

  public static SwerveDriveKinematics kinematics;
  public static SwerveDriveOdometry odometry;

  private static ChassisSpeeds speeds = new ChassisSpeeds();
  private static SwerveModuleState[] moduleStates;
  public static SwerveModulePosition[] modulePositions;
  
  public static Wheel frontRight = new Wheel(Constants.frontRightDriveCANID,
  Constants.frontRightSteerCANID, Constants.frontWheelPosition, Constants.rightWheelPosition);
  public static Wheel frontLeft = new Wheel(Constants.frontLeftDriveCANID,
  Constants.frontLeftSteerCANID, Constants.frontWheelPosition, Constants.leftWheelPosition);
  public static Wheel backLeft = new Wheel(Constants.backLeftDriveCANID,
  Constants.backLeftSteerCANID, Constants.backWheelPosition, Constants.leftWheelPosition);
  public static Wheel backRight = new Wheel(Constants.backRightDriveCANID, 
  Constants.backRightSteerCANID, Constants.backWheelPosition, Constants.rightWheelPosition);

  /**
   * Initialize all variables, objects, and methods for a created SwerveDrive object.
   */
  public SwerveDrive() {
    // Initialize and zero gyro
    gyro.calibrate();
    resetGyro();
    
    // Pass in the reported positions from each module, to prevent any weird offsets
    modulePositions = new SwerveModulePosition[] {frontRight.getPosition(),
        frontLeft.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    
    // Pass in locations of wheels relative to the center of the robot
    // These are later used in the backend, likely to find the angles the wheels need to rotate to when the robot spins 
    kinematics = new SwerveDriveKinematics(frontRight.location,
        frontLeft.location, backLeft.location, backRight.location);
    
    // Pass in wheel module locations, as well as initial robot angle and position for field oriented drive
    odometry = new SwerveDriveOdometry(kinematics, gyroRotation2d,
        modulePositions, new Pose2d(0, 0, new Rotation2d()));

    AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getRobotRelativeSpeeds,
        this::driveRobotRelative, new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),Constants.swerveMaxVelocity,
        Math.sqrt(Math.pow(Constants.frontWheelPosition, 2) + Math.pow(Constants.rightWheelPosition, 2)),
        new ReplanningConfig()), this);
  }

  @Override
  public void periodic() {
    gyroRotation2d = gyro.getRotation2d().unaryMinus();
  }

  public static void resetGyro() {
    gyro.reset();
    gyroRotation2d = gyro.getRotation2d().unaryMinus();
  }

  /**
   * Call the updatePIDValues() method for each wheel module.
   * 
   * @param DFF
   *     Drive Feed-Forward value
   * @param DP
	 *     Drive Proportional value
   * @param DI
	 *     Drive Integral value
   * @param DD
	 *     Drive Derivative value
   * @param SFF
	 *     Steer Feed Forward value
   * @param SP
	 *     Steer Proportional value
   * @param SI
	 *     Steer Integral value
   * @param SD
	 *     Steer Derivative value
   */
  public static void updatePIDValues(double DFF, double DP, double DI, double DD, double SFF, double SP, double SI, double SD) {
    frontRight.updatePIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
    frontLeft.updatePIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
    backLeft.updatePIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
    backRight.updatePIDValues(DFF, DP, DI, DD, SFF, SP, SI, SD);
  }

  /**
   * Do the math to calculate the speeds and angles for each wheel on the swerve drive.
   * 
   * @param x
	 *     Desired X speed of the robot from -1 to 1
   * @param y
	 *     Desired Y speed of the robot from -1 to 1
   * @param spin
	 *     Desired rotational speed of the robot from -1 to 1
   * @param xyMod
   *     Number to multiply the translation speed of the robot by, to modify speed while driving
   * @param spinMod
   *     Number to multiply the rotation speed of the robot by, to modify speed while driving
   */
  public static void calculateSpeedsAndAngles(double x, double y, double spin, double xyMod, double rotationMod, boolean isFieldOriented) {
    // Set the desired speeds for the robot, we also pass in the gyro angle for field oriented drive
    if (isFieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds((y * xyMod), (x * xyMod), (spin * rotationMod), gyroRotation2d);
    } else {
      speeds = new ChassisSpeeds((y * xyMod), (x * xyMod), (spin * rotationMod));
    }

    // Convert overall robot speeds and angle into speeds and angles for each wheel module, referred to as module states
    moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    frontLeft.moduleState = moduleStates[0];

    // Front right module state
    frontRight.moduleState = moduleStates[1];

    // Back left module state
    backLeft.moduleState = moduleStates[2];

    // Back right module state
    backRight.moduleState = moduleStates[3];
  }

  /**
   * Do the math to optimize wheel angles, and output to the wheel modules.
   */
  public static void optimizeAndSetOutputs() {
    System.out.println(odometry.getPoseMeters().getX() + ", " + odometry.getPoseMeters().getY());

    // Do math to get multiple variables out of the encoder position
    frontLeft.setEncoderVariables();
    frontRight.setEncoderVariables();
    backLeft.setEncoderVariables();
    backRight.setEncoderVariables();
    
    // Do math for swerve drive that is identical between all wheel modules, and then send the angle and speed to the wheels
    frontRight.optimizeAndCalculateVariables();
    frontLeft.optimizeAndCalculateVariables();
    backLeft.optimizeAndCalculateVariables();
    backRight.optimizeAndCalculateVariables();

    // Update Odometry, so the robot knows its position on the field
    modulePositions = new SwerveModulePosition[] {frontRight.getPosition(),
        frontLeft.getPosition(), backLeft.getPosition(), backRight.getPosition()};

    odometry.update(gyroRotation2d, modulePositions);

    frontRight.setOutputs();
    frontLeft.setOutputs();
    backLeft.setOutputs();
    backRight.setOutputs();
  }

  /**
   * Get the robot pose from the Odometry.
   * 
   * @return The Pose2d of the robot
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyroRotation2d, modulePositions, pose);
  }

  /**
   * Get the robot angle from the Odometry.
   * 
   * @return The Rotation2d of the robot
   */
  public Rotation2d getRotation() {
    return gyroRotation2d;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(moduleStates);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    calculateSpeedsAndAngles(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, 1, 1, false);;
  }

  /**
   * Set the module state for each wheel, essentially passing the desired speed and angle to each wheel.
   * 
   * @param DesiredStates The desired state of all the wheels
   */
  public static void setModuleStates(SwerveModuleState[] DesiredStates) {    
    SwerveDriveKinematics.desaturateWheelSpeeds(DesiredStates, Constants.swerveMaxVelocity);
    moduleStates = DesiredStates;
    // Front left module state
    frontLeft.moduleState = new SwerveModuleState(moduleStates[0].speedMetersPerSecond,
        new Rotation2d(moduleStates[0].angle.getRadians()));
    // Front right module state
    frontRight.moduleState = new SwerveModuleState(moduleStates[1].speedMetersPerSecond,
        new Rotation2d(moduleStates[1].angle.getRadians()));
    // Back left module state
    backLeft.moduleState = new SwerveModuleState(moduleStates[2].speedMetersPerSecond,
        new Rotation2d(moduleStates[3].angle.getRadians()));
    // Back right module state
    backRight.moduleState = new SwerveModuleState(moduleStates[3].speedMetersPerSecond,
        new Rotation2d(moduleStates[3].angle.getRadians()));

    optimizeAndSetOutputs();
  }
}
