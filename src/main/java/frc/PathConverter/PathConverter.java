// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.PathConverter;

import java.io.*;
import java.util.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.SwerveSubsystem.SwerveDrive;
import frc.robot.Constants;

public class PathConverter extends SubsystemBase {
  private static TrajectoryLoader TrajectoryLoader;
  private static List<SendableChooser<String>> AutoChoosers;
  private static int AutoChooserListID;
  private static String[] AutoNames;
  private static String AutoFile;
  private static File TrajFile;
  private static TrajectoryConfig TrajConfig;
  private static Trajectory Trajectory;
  private static SwerveDriveKinematicsConstraint SwerveDriveMaxSpeed;
  private static Constraints PIDConstraints;
  private static Scanner AutoReader;
  private static List<String> Lines;
  private static List<String> CurrentLine;
  private static List<String> FileOrder;
  public static List<String> AutoOrder;
  private static List<Translation2d> Translation2ds;
  private static List<Rotation2d> Rotation2ds;
  private static List<Pose2d> Pose2ds;
  private static List<Translation2d> MiddlePoints;
  public static List<SwerveControllerCommand> SwerveControllerCommands;
  private static int StartIndex;

  public static class TrajectoryLoader extends CommandBase {
    @Override
    public boolean runsWhenDisabled() {
      return true;
    }

    @Override
    public void initialize() {
      AutoFile = "None";
    }
  
    @Override
    public void execute() {
      if (AutoFile != AutoChoosers.get(AutoChooserListID).getSelected()) {
        AutoFile = AutoChoosers.get(AutoChooserListID).getSelected();
        System.out.println(AutoFile);
        try {
          initTrajectory();
        } catch (FileNotFoundException e) {
          System.out.println("AUTO NOT FOUND");
        }
      }

      if ((DriverStation.getAlliance() == DriverStation.Alliance.Red & AutoChooserListID != 0) || (DriverStation.getAlliance() == DriverStation.Alliance.Blue & AutoChooserListID != 1)) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
          AutoChooserListID = 0;
        }
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
          AutoChooserListID = 1;
        }
        SmartDashboard.putData("AutoChooser", AutoChoosers.get(AutoChooserListID));
      }
    }
  }

  public PathConverter() {
  }

  public static void init() {
    Lines = new ArrayList<String>();
    CurrentLine = new ArrayList<String>();
    FileOrder = new ArrayList<String>();
    AutoOrder = new ArrayList<String>();
    Translation2ds = new ArrayList<Translation2d>();
    Rotation2ds = new ArrayList<Rotation2d>();
    Pose2ds = new ArrayList<Pose2d>();
    MiddlePoints = new ArrayList<Translation2d>();
    SwerveControllerCommands = new ArrayList<SwerveControllerCommand>();

    AutoChoosers = Arrays.asList(new SendableChooser<String>(), new SendableChooser<String>());
    AutoNames = Filesystem.getDeployDirectory().toPath().resolve("output/paths").toFile().list();
    for (Integer Index = 0; Index <= AutoNames.length - 1; Index++) {
      // Create auto options for each alliance
      if (AutoNames[Index].toString().contains("Red")) {
        AutoChoosers.get(0).addOption(AutoNames[Index], AutoNames[Index]);
      }
      else if (AutoNames[Index].toString().contains("Blue")) {
        AutoChoosers.get(1).addOption(AutoNames[Index], AutoNames[Index]);
      }
    }
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      AutoChooserListID = 0;
    }
    else {
      AutoChooserListID = 1;
    }
    AutoChoosers.get(0).setDefaultOption("RedTest", "RedTest");
    AutoChoosers.get(1).setDefaultOption("BlueTest", "BlueTest");

    SmartDashboard.putData("AutoChooser", AutoChoosers.get(AutoChooserListID));

    TrajectoryLoader = new TrajectoryLoader();
    TrajectoryLoader.schedule();
  }

  private static void initTrajectory() throws FileNotFoundException {
    TrajFile = Filesystem.getDeployDirectory().toPath().resolve("output/paths/" + AutoFile).toFile();
    AutoReader = new Scanner(TrajFile);
    SwerveDriveMaxSpeed = new SwerveDriveKinematicsConstraint(SwerveDrive.Kinematics, Constants.SwerveMaxVelocity);
    PIDConstraints = new Constraints(Constants.SwerveMaxVelocity, Constants.SwerveMaxAcceleration);
    TrajConfig = new TrajectoryConfig(Constants.SwerveMaxVelocity, Constants.SwerveMaxAcceleration).setKinematics(SwerveDrive.Kinematics).addConstraint(SwerveDriveMaxSpeed);
    Lines.clear();
    CurrentLine.clear();
    Translation2ds.clear();
    Rotation2ds.clear();
    Pose2ds.clear();
    SwerveControllerCommands.clear();
    FileOrder.clear();
    AutoOrder.clear();
    SwerveDrive.Gyro.reset();

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
    SwerveDrive.Odometry.resetPosition(SwerveDrive.GyroRotation2d, new SwerveModulePosition[] {SwerveDrive.FrontRight.getPosition(), SwerveDrive.FrontLeft.getPosition(), SwerveDrive.BackLeft.getPosition(), SwerveDrive.BackRight.getPosition()}, Pose2ds.get(0));

    System.out.println(FileOrder);

    // Create all required SwerveControllerCommands, as well as a roadmap for what to do at each step of auto
    for (Integer Index = 0; Index <= FileOrder.size() - 1; Index++) {
      // If the next command is to move, create a SwerveControllerCommand for every point up to the next non-move command
      if (FileOrder.get(Index).equals("Move")) {
        // Store the starting index, since this is the beginning point of the move, then increment the index
        StartIndex = Index++;
        // Create the list of midpoints
        if (Index <= FileOrder.size() - 2) {
          while (Index <= FileOrder.size() - 2 & FileOrder.get(Index).equals("Move")) {
            MiddlePoints.add(Translation2ds.get(Index++));
          }
          if (Index <= FileOrder.size() - 1) {
            if (FileOrder.get(Index).equals("Move")) {
              MiddlePoints.add(Translation2ds.get(Index++));
            }
          }
        }
        System.out.println(MiddlePoints.size());
        // Generate the trajectory, using the StartIndex for the starting position, the MiddlePoints list we just created, and the current index as the endpoint
        Trajectory = TrajectoryGenerator.generateTrajectory(Pose2ds.get(StartIndex), MiddlePoints, Pose2ds.get(Index), TrajConfig);
        // Generate the SwerveControllerCommand, and put it in the SwerveControllerCommandslist
        SwerveControllerCommands.add(new SwerveControllerCommand(Trajectory, SwerveDrive::getPose, SwerveDrive.Kinematics, new HolonomicDriveController(new PIDController(3, 0, 0), new PIDController(-3, 0, 0), new ProfiledPIDController(1.5, 0, 0, PIDConstraints)), SwerveDrive::setModuleStates));
        // Decrement the index in preparation for the for loop to increment it
        Index--;
        MiddlePoints.clear();
      }
      // Add the command to the AutoOrder list, which will act as a roadmap for Auto
      AutoOrder.add(FileOrder.get(Index));
    }
    System.out.println(AutoOrder);
  }

  private static void addPointToLists() {
    // Add the Translation2d of the point to the list
    Translation2ds.add(new Translation2d(Double.parseDouble(CurrentLine.get(0)),Double.parseDouble(CurrentLine.get(1))));
    // Add the Rotation2d of the point to the list
    Rotation2ds.add(new Rotation2d(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2)))));
    System.out.println(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))));
    // Also add the Pose2d of the point to the list, for the endpoints
    Pose2ds.add(new Pose2d(Translation2ds.get(Translation2ds.size() - 1), Rotation2ds.get(Rotation2ds.size() - 1)));
  }
}