// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.function.Supplier;

import org.ejml.ops.ReadCsv;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private File Trajectory;
  private Scanner AutoReader;
  private List<String> Lines;
  private List<String> CurrentLine;
  private List<Double> XPositions;
  private List<Double> YPositions;
  private List<Double> Angles;
  private List<Boolean> Reversed;
  private List<Integer> AutoStage;
  private Integer Index;
  private PIDController XPIDController;
  private PIDController YPIDController;
  private PIDController AnglePIDController;

  public Autonomous() {
    Limelight = new Tracking();
    Lines = new ArrayList<String>();
    CurrentLine = new ArrayList<String>();
    XPositions = new ArrayList<Double>();
    YPositions = new ArrayList<Double>();
    Angles = new ArrayList<Double>();
    Reversed = new ArrayList<Boolean>();
    AutoStage = new ArrayList<Integer>();
  }

  public void initTrajectory(String AutoFile) throws FileNotFoundException {
    TrajectoryPath = ("/deploy/" + AutoFile);
    Trajectory = new File(TrajectoryPath);
    AutoReader = new Scanner(Trajectory);

    if (AutoReader.hasNextLine()) {
      AutoReader.nextLine();
    }
    while (AutoReader.hasNextLine()) {
      Lines.add(AutoReader.nextLine());
    }
    for (Index = 0; Index <= Lines.size(); Index++) {
      CurrentLine.addAll(Arrays.asList(Lines.get(Index).split(",")));
      XPositions.add(Double.parseDouble(CurrentLine.get(0)));
      YPositions.add(Double.parseDouble(CurrentLine.get(1)));
      if (TrajectoryPath.contains("Red")) {
        Angles.add(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))) - Math.PI);
      }
      if (TrajectoryPath.contains("Blue")) {
        Angles.add(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))));
      }
      Reversed.add(Boolean.getBoolean(CurrentLine.get(5)));
      AutoStage.add(Integer.getInteger(CurrentLine.get(6)));
      CurrentLine.clear();
    }
    AutoReader.close();
    Index = 0;
    AnglePIDController.enableContinuousInput(0, (2 * Math.PI));
  }

  public void runAutonomous() {
    if (Index == 0) {
      SwerveDrive.Odometry.resetPosition(new Rotation2d(Angles.get(Index)), SwerveDrive.ModulePositions, new Pose2d(XPositions.get(Index), YPositions.get(Index), new Rotation2d(Angles.get(Index))));
      Index = 1;
    }
    if (Index >= 1) {
      
    }
  }
}
