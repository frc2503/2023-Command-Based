// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import frc.robot.Constants;

import java.io.*;

public class Autonomous extends SubsystemBase {
  private Tracking Track;
  private SwerveDrive Swerve;
  private RobotMechanisms Mechanisms;
  private TrajectoryLoader TrajectoryLoader;
  private SendableChooser<String> AutoChooser;
  private String[] AutoNames;
  private String AutoFile;
  private File TrajFile;
  private TrajectoryConfig TrajConfig;
  private Trajectory Trajectory;
  private SwerveDriveKinematicsConstraint SwerveDriveMaxSpeed;
  private Constraints PIDConstraints;
  private Scanner AutoReader;
  private List<String> Lines;
  private List<String> CurrentLine;
  private List<String> FileOrder;
  private List<String> AutoOrder;
  private List<Translation2d> Translation2ds;
  private List<Rotation2d> Rotation2ds;
  private List<Pose2d> Pose2ds;
  private List<Translation2d> MiddlePoints;
  private List<SwerveControllerCommand> SwerveControllerCommands;
  private Integer AutoStage;
  private Integer StartIndex;
  private Integer SwerveControllerCommandIndex;
  private Boolean IsScheduled = false;
  public Timer Timer;

  public class TrajectoryLoader extends CommandBase {
    private String PrevAuto;

    @Override
    public void initialize() {
      // Load default autonomous file
      PrevAuto = AutoChooser.getSelected();
      AutoFile = AutoChooser.getSelected();
      if (AutoChooser.getSelected() != null) {
        System.out.println(AutoFile);
        try {
          initTrajectory();
        } catch (FileNotFoundException e) {
          System.out.println("AUTO NOT FOUND");
        }
      }
    }
  
    @Override
    public void execute() {
      if (PrevAuto != AutoChooser.getSelected()) {
        AutoFile = AutoChooser.getSelected();
        System.out.println(AutoFile);
        try {
          initTrajectory();
        } catch (FileNotFoundException e) {
          System.out.println("AUTO NOT FOUND");
        }
        PrevAuto = AutoChooser.getSelected();
      }
    }
  }

  public Autonomous(SwerveDrive SwerveDrive, Tracking Tracking, RobotMechanisms RobotMechanisms) {
    Swerve = SwerveDrive;
    Track = Tracking;
    Mechanisms = RobotMechanisms;
    Lines = new ArrayList<String>();
    CurrentLine = new ArrayList<String>();
    FileOrder = new ArrayList<String>();
    AutoOrder = new ArrayList<String>();
    Translation2ds = new ArrayList<Translation2d>();
    Rotation2ds = new ArrayList<Rotation2d>();
    Pose2ds = new ArrayList<Pose2d>();
    MiddlePoints = new ArrayList<Translation2d>();
    SwerveControllerCommands = new ArrayList<SwerveControllerCommand>();
    AutoStage = 0;
    SwerveControllerCommandIndex = 0;
    Timer = new Timer();

    AutoChooser = new SendableChooser<String>();
    AutoNames = Filesystem.getDeployDirectory().toPath().resolve("output/paths").toFile().list();
    for (Integer Index = 0; Index <= AutoNames.length - 1; Index++) {
      AutoChooser.addOption(AutoNames[Index], AutoNames[Index]);
    }
    AutoChooser.setDefaultOption("BlueTest", "BlueTest");
    SmartDashboard.putData("AutoChooser", AutoChooser);

    TrajectoryLoader = new TrajectoryLoader();
    TrajectoryLoader.schedule();
  }

  public void initTrajectory() throws FileNotFoundException {
    TrajFile = Filesystem.getDeployDirectory().toPath().resolve("output/paths/" + AutoFile).toFile();
    AutoReader = new Scanner(TrajFile);
    SwerveDriveMaxSpeed = new SwerveDriveKinematicsConstraint(Swerve.Kinematics, Constants.SwerveMaxVelocity);
    PIDConstraints = new Constraints(Constants.SwerveMaxVelocity, Constants.SwerveMaxAcceleration);
    TrajConfig = new TrajectoryConfig(Constants.SwerveMaxVelocity, Constants.SwerveMaxAcceleration).setKinematics(Swerve.Kinematics).addConstraint(SwerveDriveMaxSpeed);
    Lines.clear();
    CurrentLine.clear();
    Translation2ds.clear();
    Rotation2ds.clear();
    Pose2ds.clear();
    SwerveControllerCommands.clear();
    FileOrder.clear();
    AutoOrder.clear();
    AutoStage = 0;
    Swerve.Gyro.reset();

    // Skip title line
    if (AutoReader.hasNextLine()) {
      AutoReader.nextLine();
    }
    // Parse entire auto file, and place each seperate line into an entry in the Lines list
    while (AutoReader.hasNextLine()) {
      Lines.add(AutoReader.nextLine());
    }
    AutoReader.close();
    // Pull all data entry-by-entry from the Lines list and add that data to new lists
    for (Integer Index = 0; Index <= Lines.size() - 1; Index++) {
      // Splits the next entry at each column, and adds that data to the CurrentLine list
      CurrentLine.addAll(Arrays.asList(Lines.get(Index).split(",")));
      // Skips adding the first "Move" string to the FileOrder list
      // This normalizes every series of "Move" entries to be 1 "Move" short of the actual number of points
      // This also helps because any commands on the first point will need to be done before the first move
      // So when we add the command here, it is before any "Move" entries
      if (Index == 0) {
        if (CurrentLine.size() == 7) {
          FileOrder.add(CurrentLine.get(6));
          addPointToLists();
        }
      }
      else {
        FileOrder.add("Move");
        // If there is a command at the point, add the command to the FileOrder list
        // Also adds a copy of the current point to the lists
        // This simplifies later code by having points for both the endpoint of this move and the beginning of the next
        if (CurrentLine.size() == 7) {
          FileOrder.add(CurrentLine.get(6));
          addPointToLists();
        }
      }
      
      addPointToLists();
      CurrentLine.clear();
    }
    // Set the position of the odometry to the starting position of the auto
    Swerve.Odometry.resetPosition(Swerve.GyroRotation2d, new SwerveModulePosition[] {Swerve.FrontRight.getPosition(), Swerve.FrontLeft.getPosition(), Swerve.BackLeft.getPosition(), Swerve.BackRight.getPosition()}, Pose2ds.get(0));

    System.out.println(FileOrder);

    // Create all required SwerveControllerCommands, as well as a roadmap for what to do at each step of auto
    for (Integer Index = 0; Index <= FileOrder.size() - 1; Index++) {
      // If the next command is to move, create a SwerveControllerCommand for every point up to the next non-move command
      if (FileOrder.get(Index) == "Move") {
        // Store the starting index, since this is the beginning point of the move, then increment the index
        StartIndex = Index++;
        // Create the list of midpoints
        if (Index <= FileOrder.size() - 2) {
          while (Index <= FileOrder.size() - 2 & FileOrder.get(Index) == "Move") {
            MiddlePoints.add(Translation2ds.get(Index++));
          }
        }
        // Generate the trajectory, using the StartIndex for the starting position, the MiddlePoints list we just created, and the current index as the endpoint
        Trajectory = TrajectoryGenerator.generateTrajectory(Pose2ds.get(StartIndex), MiddlePoints, Pose2ds.get(Index), TrajConfig);
        // Generate the SwerveControllerCommand, and put it in the SwerveControllerCommandslist
        SwerveControllerCommands.add(new SwerveControllerCommand(Trajectory, Swerve::getPose, Swerve.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), Swerve::setModuleStates, Swerve));
        // Decrement the index in preparation for the for loop to increment it
        Index--;
        MiddlePoints.clear();
      }
      // Add the command to the AutoOrder list, which will act as a roadmap for Auto
      AutoOrder.add(FileOrder.get(Index));
    }
    System.out.println(AutoOrder);
  }

  private void addPointToLists() {
    // Add the Translation2d of the point to the list
    Translation2ds.add(new Translation2d(Double.parseDouble(CurrentLine.get(0)),Double.parseDouble(CurrentLine.get(1))));
    // Check what alliance the auto is for, since Pathweaver doesn't take into account the alliance the robot is on
    if (AutoFile.contains("Red")) {
      // Add the Rotation2d of the point to the list, and invert it to solve the previously stated issue
      Rotation2ds.add(new Rotation2d(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))) + Math.PI));
    }
    if (AutoFile.contains("Blue")) {
      // Add the Rotation2d of the point to the list
      Rotation2ds.add(new Rotation2d(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2)))));
    }
    // Also add the Pose2d of the point to the list, for the endpoints
    Pose2ds.add(new Pose2d(Translation2ds.get(Translation2ds.size() - 1), Rotation2ds.get(Rotation2ds.size() - 1)));
  }

  public void runAutonomous() {
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage).equals("Move")) {
        if (!IsScheduled) {
          System.out.println("Move");
          SwerveControllerCommands.get(SwerveControllerCommandIndex).andThen(() -> Swerve.stop()).schedule();
          IsScheduled = true;
        }
        if (SwerveControllerCommands.get(SwerveControllerCommandIndex).isFinished()) {
          AutoStage++;
          SwerveControllerCommandIndex++;
          IsScheduled = false;
        }
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage).equals("Grab Cone")) {
        System.out.println("Grab Cone");
        Mechanisms.DesiredState = "Grab";
        Mechanisms.openGrabber();
        if(Mechanisms.isAtDesiredState()) {
          if(Timer.get() > 0) { //If we are grabbing the cone, we don't want to open grabber, nor keep moving
            Track.centerOnCone();
          }
          if(Math.abs(Track.IntakeTargetOffsetV.getDouble(0)) <= 20) {//moved towards cone
            if(Timer.get() <= 0.0) {
              Mechanisms.closeGrabber();
              Timer.start();
            }
            if(Timer.get() >= 0.1) { //waited for grabber to close
              Mechanisms.DesiredState = "High";
              AutoStage++;
              Timer.stop();
              Timer.reset();
            }
          }
        }
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage).equals("Grab Cube")) {
        System.out.println("Grab Cube");
        Mechanisms.DesiredState = "Grab";
        Mechanisms.openGrabber();
        if(Mechanisms.isAtDesiredState()) {
          if(Timer.get() > 0) { //If we are grabbing the cone, we don't want to open grabber, nor keep moving
            Track.centerOnCube();
          }
          if(Math.abs(Track.IntakeTargetOffsetV.getDouble(0)) <= 20) {//moved towards cone
            if(Timer.get() <= 0.0) {
              Mechanisms.closeGrabber();
              Timer.start();
            }
            if(Timer.get() >= 0.1) { //waited for grabber to close
              Mechanisms.DesiredState = "High";
              AutoStage++;
              Timer.stop();
              Timer.reset();
            }
          }
        }
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage).equals("Place Cone")) {
        System.out.println("Place Cone");
        Mechanisms.DesiredState = "Place1";
        System.out.println("Arm Angle: " + Mechanisms.ArmAngle.getEncoder().getPosition());
        if(Mechanisms.isAtDesiredState()) {
          if(Timer.get() <= 0.0) {
            Mechanisms.openGrabber();
            Timer.start();
          }
          if(Timer.get() >= 0.2) { //wait for grabber to open
            Mechanisms.DesiredState = "Grab";
            AutoStage++;
            Timer.stop();
            Timer.reset();
          }
          // Track.centerOnPole();
          // if(Math.abs(Track.ArmTargetOffsetH.getDouble(0)) <= 20) {
          //   if(Timer.get() <= 0.0) {
          //     Mechanisms.openGrabber();
          //     Timer.start();
          //   }
          //   if(Timer.get() >= 0.2) { //waited for grabber to open
          //     Mechanisms.DesiredState = "Stowed";
          //     AutoStage++;
          //     Timer.stop();
          //     Timer.reset();
          //   }
        }
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage).equals("Place Cube")) {
        System.out.println("Place Cube");
        Mechanisms.DesiredState = "Place1";
        if(Mechanisms.isAtDesiredState()) {
          Track.centerOnPlatform();
          if(Math.abs(Track.ArmTargetOffsetH.getDouble(0)) <= 20) {
            Mechanisms.openGrabber();
            Mechanisms.DesiredState = "High";
            AutoStage++;
          }
        }
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage).equals("Charge")) {
        System.out.println("Charge");
        AutoStage++;
      }
    }
  }
}