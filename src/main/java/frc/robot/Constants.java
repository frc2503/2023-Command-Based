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
    // CAN IDs for all swerve motor controllers
    public static int FrontRightDriveCANID = 1;
    public static int FrontRightSteerCANID = 5;

    public static int FrontLeftDriveCANID = 2;
    public static int FrontLeftSteerCANID = 6;

    public static int BackLeftDriveCANID = 3;
    public static int BackLeftSteerCANID = 7;

    public static int BackRightDriveCANID = 4;
    public static int BackRightSteerCANID = 8;


    // PID values for swerve PID loops
    public static double DriveFeedForward = 0.000175;
    public static double DriveProportional = 0.00001;
    public static double DriveIntegral = 0.0000004;
    public static double DriveDerivative = 0;

    public static double SteerFeedForward = 0;
    public static double SteerProportional = 8;
    public static double SteerIntegral = 0.01;
    public static double SteerDerivative = 0.01;
}
