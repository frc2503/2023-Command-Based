// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tracking extends SubsystemBase {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public static NetworkTable armLimelight = inst.getTable("limelight-arm");
  public static NetworkTableEntry armHasTarget = armLimelight.getEntry("tv");
  public static NetworkTableEntry armTargetOffsetH = armLimelight.getEntry("tx");
  public static NetworkTableEntry armTargetOffsetV = armLimelight.getEntry("ty");
  public static NetworkTableEntry armTargetArea = armLimelight.getEntry("ta");
  public static NetworkTableEntry armTargetSkew = armLimelight.getEntry("ts");
  public static NetworkTableEntry armPipeline = armLimelight.getEntry("pipeline");
  
  public static NetworkTable intakeLimelight = inst.getTable("limelight-intake");
  public static NetworkTableEntry intakeHasTarget = intakeLimelight.getEntry("tv");
  public static NetworkTableEntry intakeTargetOffsetH = intakeLimelight.getEntry("tx");
  public static NetworkTableEntry intakeTargetOffsetV = intakeLimelight.getEntry("ty");
  public static NetworkTableEntry intakeTargetArea = intakeLimelight.getEntry("ta");
  public static NetworkTableEntry intakeTargetSkew = intakeLimelight.getEntry("ts");
  public static NetworkTableEntry intakePipeline = intakeLimelight.getEntry("pipeline");
  private static Constraints PIDConstraints = new Constraints(Constants.swerveMaxVelocity, .5);
  public static ProfiledPIDController PID = new ProfiledPIDController(1, 0, 0, PIDConstraints);

  /**
   * Select the proper pipeline to target a pole
   */
  public static void selectPoleTargeting() {
    armPipeline.setValue(0);
  }

  /**
   * Select the proper pipeline to target a platform
   */
  public static void selectPlatformTargeting() {
    armPipeline.setValue(1);
  }

  /**
   * Select the proper pipeline to target a platform
   */
  public static void selectConeTargeting() {
    intakePipeline.setValue(0);
  }

  /**
   * Select the proper pipeline to target a platform
   */
  public static void selectCubeTargeting() {
    intakePipeline.setValue(1);
  }
}
