// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.function.Supplier;

import org.ejml.ops.ReadCsv;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
import java.io.*;

public class Autonomous extends SubsystemBase {
  public Tracking Limelight;
  private SwerveDrive SwerveDrive;
  private String TrajectoryPath;
  private File TrajectoryFile;
  private TrajectoryConfig TrajectoryConfig;
  private Trajectory PlaceFirstObjectTrajectory;
  private Trajectory GetSecondObjectTrajectory;
  private Trajectory PlaceSecondObjectTrajectory;
  private Trajectory ToChargerTrajectory;
  private SwerveDriveKinematicsConstraint SwerveDriveMaxSpeed;
  private Constraints PIDConstraints;
  private Subsystem[] RequiredSubsystems;
  private Scanner AutoReader;
  private List<String> Lines;
  private List<String> CurrentLine;
  private List<Double> XPositions;
  private List<Double> YPositions;
  private List<Double> Angles;
  private Boolean HasChargerAuto;
  private SwerveControllerCommand PlaceFirstObject;
  private SwerveControllerCommand GetSecondObject;
  private SwerveControllerCommand PlaceSecondObject;
  private SwerveControllerCommand ToCharger;
  private Integer AutoStage;

  public Autonomous() {
    Limelight = new Tracking();
    Lines = new ArrayList<String>();
    CurrentLine = new ArrayList<String>();
    XPositions = new ArrayList<Double>();
    YPositions = new ArrayList<Double>();
    Angles = new ArrayList<Double>();
    AutoStage = 0;
  }

  public void initTrajectory(String AutoFile) throws FileNotFoundException {
    TrajectoryPath = ("/deploy/" + AutoFile);
    TrajectoryFile = new File(TrajectoryPath);
    AutoReader = new Scanner(TrajectoryFile);
    SwerveDriveMaxSpeed = new SwerveDriveKinematicsConstraint(SwerveDrive.Kinematics, 2);
    PIDConstraints = new Constraints(2, 2);
    TrajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(SwerveDrive.Kinematics).addConstraint(SwerveDriveMaxSpeed);

    if (AutoReader.hasNextLine()) {
      AutoReader.nextLine();
    }
    while (AutoReader.hasNextLine()) {
      Lines.add(AutoReader.nextLine());
    }
    for (Integer Index = 0; Index <= Lines.size(); Index++) {
      CurrentLine.addAll(Arrays.asList(Lines.get(Index).split(",")));
      XPositions.add(Double.parseDouble(CurrentLine.get(0)));
      YPositions.add(Double.parseDouble(CurrentLine.get(1)));
      if (TrajectoryPath.contains("Red")) {
        Angles.add(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))) - Math.PI);
      }
      if (TrajectoryPath.contains("Blue")) {
        Angles.add(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))));
      }
      if (Index == 6) {
        HasChargerAuto = true;
      }
      CurrentLine.clear();
    }
    AutoReader.close();
    SwerveDrive.Odometry.resetPosition(new Rotation2d(Angles.get(0)), new SwerveModulePosition[] {SwerveDrive.FrontRight.getPosition(), SwerveDrive.FrontLeft.getPosition(), SwerveDrive.BackLeft.getPosition(), SwerveDrive.BackRight.getPosition()}, new Pose2d(new Translation2d(XPositions.get(0), YPositions.get(0)), new Rotation2d(Angles.get(0))));

    PlaceFirstObjectTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(0), YPositions.get(0)), new Rotation2d(Angles.get(0))), List.of(), new Pose2d(new Translation2d(XPositions.get(1), YPositions.get(1)), new Rotation2d(Angles.get(1))), TrajectoryConfig);
    GetSecondObjectTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(1), YPositions.get(1)), new Rotation2d(Angles.get(1))), List.of(new Translation2d(XPositions.get(2), YPositions.get(2))), new Pose2d(new Translation2d(XPositions.get(3), YPositions.get(3)), new Rotation2d(Angles.get(3))), TrajectoryConfig);
    PlaceSecondObjectTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(3), YPositions.get(3)), new Rotation2d(Angles.get(3))), List.of(new Translation2d(XPositions.get(4), YPositions.get(4))), new Pose2d(new Translation2d(XPositions.get(5), YPositions.get(5)), new Rotation2d(Angles.get(5))), TrajectoryConfig);
    ToChargerTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(5), YPositions.get(5)), new Rotation2d(Angles.get(5))), List.of(), new Pose2d(new Translation2d(XPositions.get(6), YPositions.get(6)), new Rotation2d(Angles.get(6))), TrajectoryConfig);

    PlaceFirstObject = new SwerveControllerCommand(PlaceFirstObjectTrajectory, SwerveDrive::getPose, SwerveDrive.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), SwerveDrive::setModuleStates, RequiredSubsystems);
    GetSecondObject = new SwerveControllerCommand(GetSecondObjectTrajectory, SwerveDrive::getPose, SwerveDrive.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), SwerveDrive::setModuleStates, RequiredSubsystems);
    PlaceSecondObject = new SwerveControllerCommand(PlaceSecondObjectTrajectory, SwerveDrive::getPose, SwerveDrive.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), SwerveDrive::setModuleStates, RequiredSubsystems);
    ToCharger = new SwerveControllerCommand(ToChargerTrajectory, SwerveDrive::getPose, SwerveDrive.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), SwerveDrive::setModuleStates, RequiredSubsystems);
  }

  public void runAutonomous() {
    if (AutoStage == 0) {
      PlaceFirstObject.andThen(() -> SwerveDrive.swerveDrive(0.0, 0.0, 0.0, 1.0, 1.0));
      if (PlaceFirstObject.isFinished()) {
        AutoStage = 1;
      }
    }
    if (AutoStage == 1) {
      //Extend arm
      //Line up with Limelight
      //Drop object
      //Retract arm
      AutoStage = 2;
    }
    if (AutoStage == 2) {
      GetSecondObject.andThen(() -> SwerveDrive.swerveDrive(0.0, 0.0, 0.0, 1.0, 1.0));
      if (GetSecondObject.isFinished()) {
        AutoStage = 3;
      }
    }
    if (AutoStage == 3) {
      //Extend intake
      //Start motors
      //Pick up with Limelight
      //Stop motors
      //Retract intake
      AutoStage = 4;
    }
    if (AutoStage == 4) {
      PlaceSecondObject.andThen(() -> SwerveDrive.swerveDrive(0.0, 0.0, 0.0, 1.0, 1.0));
      if (PlaceSecondObject.isFinished()) {
        AutoStage = 5;
      }
    }
    if (AutoStage == 5) {
      //Extend arm
      //Line up with Limelight
      //Drop object
      //Retract arm
      AutoStage = 6;
    }
    if (AutoStage == 6 & HasChargerAuto) {
      ToCharger.andThen(() -> SwerveDrive.swerveDrive(0.0, 0.0, 0.0, 1.0, 1.0));
      if (ToCharger.isFinished()) {
        AutoStage = 7;
      }
    }
    if (AutoStage == 7) {
      //Auto balance on charger
    }
  }
}