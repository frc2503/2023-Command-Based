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
import frc.SwerveSubsystem.SwerveDrive;

public class Tracking extends SubsystemBase {
  public static NetworkTableInstance Inst;
  public static NetworkTable ArmLimelight;
  public static NetworkTableEntry ArmHasTarget;
  public static NetworkTableEntry ArmTargetOffsetH;
  public static NetworkTableEntry ArmTargetOffsetV;
  public static NetworkTableEntry ArmTargetArea;
  public static NetworkTableEntry ArmTargetSkew;
  public static NetworkTableEntry ArmPipeline;
  
  public static NetworkTable IntakeLimelight;
  public static NetworkTableEntry IntakeHasTarget;
  public static NetworkTableEntry IntakeTargetOffsetH;
  public static NetworkTableEntry IntakeTargetOffsetV;
  public static NetworkTableEntry IntakeTargetArea;
  public static NetworkTableEntry IntakeTargetSkew;
  public static NetworkTableEntry IntakePipeline;
  private static Constraints PIDConstraints;
  private static ProfiledPIDController PID;

  public Tracking() {
  }

  public static void init() {
    Inst = NetworkTableInstance.getDefault();
    ArmLimelight = Inst.getTable("limelight-arm");
    ArmHasTarget = ArmLimelight.getEntry("tv");
    ArmTargetOffsetH = ArmLimelight.getEntry("tx");
    ArmTargetOffsetV = ArmLimelight.getEntry("ty");
    ArmTargetArea = ArmLimelight.getEntry("ta");
    ArmTargetSkew = ArmLimelight.getEntry("ts");
    ArmPipeline = ArmLimelight.getEntry("pipeline");

    IntakeLimelight = Inst.getTable("limelight-intake");
    IntakeHasTarget = IntakeLimelight.getEntry("tv");
    IntakeTargetOffsetH = IntakeLimelight.getEntry("tx");
    IntakeTargetOffsetV = IntakeLimelight.getEntry("ty");
    IntakeTargetArea = IntakeLimelight.getEntry("ta");
    IntakeTargetSkew = IntakeLimelight.getEntry("ts");
    IntakePipeline = IntakeLimelight.getEntry("pipeline");
    PIDConstraints = new Constraints(1, .5);
    PID = new ProfiledPIDController(1, 0, 0, PIDConstraints);
  }

  public static void centerOnPole() {
    ArmPipeline.setValue(0);
    if (ArmTargetOffsetH.getDouble(0) >= 1) {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(ArmTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0);
    }
    else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(ArmTargetOffsetH.getDouble(0), 0.0), -PID.calculate(ArmTargetOffsetV.getDouble(0), 0.0), 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }
  public static void centerOnPlatform() {
    ArmPipeline.setValue(1);
    if (ArmTargetOffsetH.getDouble(0) >= 1) {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(ArmTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0);
    }
    else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(ArmTargetOffsetH.getDouble(0), 0.0), -PID.calculate(ArmTargetOffsetV.getDouble(0), 0.0), 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }
  public static void centerOnCone() {
    IntakePipeline.setValue(0);
    if(Math.abs(IntakeTargetOffsetH.getDouble(0)) >= 1) { //If centered on the cone with deadzone, move forward. Default prevents move
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(IntakeTargetOffsetH.getDouble(0), 0.0), 0, 0.0, .1, 0.0);
    } else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(IntakeTargetOffsetH.getDouble(0), 0.0), 1, 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }
  public static void centerOnCube() {
    IntakePipeline.setValue(1);
    if(Math.abs(IntakeTargetOffsetH.getDouble(0)) >= 1) { //If centered on the cube with deadzone, move forward. Default prevents move
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(IntakeTargetOffsetH.getDouble(0), 0.0), 0, 0.0, .1, 0.0);
    } else {
      SwerveDrive.calculateSpeedsAndAngles(-PID.calculate(IntakeTargetOffsetH.getDouble(0), 0.0), 1, 0.0, .1, 0.0);
    }
    SwerveDrive.optimizeAndSetOutputs();
  }
}
