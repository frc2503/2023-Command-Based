// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int rightStickPort = 1;
  public static final int leftStickPort = 0;

  // Distance from center for wheels. Currently assumes rectangular wheel arrangement
  public static final double frontWheelPosition = 0.2604;
  public static final double backWheelPosition = -0.2604;
  public static final double rightWheelPosition = -0.2786;
  public static final double leftWheelPosition = 0.2786;

  // CAN IDs for all swerve motor controllers
  public static final int frontRightDriveCANID = 1;
  public static final int frontRightSteerCANID = 5;

  public static final int frontLeftDriveCANID = 2;
  public static final int frontLeftSteerCANID = 6;

  public static final int backLeftDriveCANID = 3;
  public static final int backLeftSteerCANID = 7;

  public static final int backRightDriveCANID = 4;
  public static final int backRightSteerCANID = 8;

  public static final int armExtendCANID = 11;
  public static final int armAngleCANID = 10;


  // PID values for swerve PID loops
  public static final double driveFeedForward = 0.000175;
  public static final double driveProportional = 0.00001;
  public static final double driveIntegral = 0.0000004;
  public static final double driveDerivative = 0;

  public static final double steerFeedForward = 0;
  public static final double steerProportional = 8;
  public static final double steerIntegral = 0.01;
  public static final double steerDerivative = 0.01;

  // Charger PID values
  public static final double chargeProportional = 1;
  public static final double chargeIntegral = 1;
  public static final double chargeDerivative = 1;

  // Number outputted by the encoders after 1 revolution
  public static final int driveEncoderCountsPerRevolution = 42;
  public static final int steerEncoderCountsPerRevolution = 1024;

  // Swerve drive maximum velocity in m/s
  public static final double swerveMaxVelocity = 1;

  // Swerve drive maximum acceleration in m/s²
  public static final double swerveMaxAcceleration = 1;
}