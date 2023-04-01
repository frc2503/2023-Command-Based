// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.PathConverter.PathConverter;
import frc.SwerveSubsystem.SwerveDrive;

public class Autonomous extends SubsystemBase {
  public static int AutoStage;
  private static int SwerveControllerCommandIndex;
  private static Boolean IsScheduled;
  public static Timer Timer;

  public Autonomous() {}

  public static void init() {
    AutoStage = 0;
    SwerveControllerCommandIndex = 0;
    IsScheduled = false;
    Timer = new Timer();
  }

  public static void runAutonomous() {
    if (AutoStage <= PathConverter.AutoOrder.size() - 1) {
      if (PathConverter.AutoOrder.get(AutoStage).equals("Move")) {
        if (!IsScheduled) {
          System.out.println("Move");
          PathConverter.SwerveControllerCommands.get(SwerveControllerCommandIndex).andThen(() -> SwerveDrive.stop()).schedule();
          IsScheduled = true;
        }
        if (PathConverter.SwerveControllerCommands.get(SwerveControllerCommandIndex).isFinished()) {
          AutoStage++;
          SwerveControllerCommandIndex++;
          IsScheduled = false;
        }
      }
    }
    if (AutoStage <= PathConverter.AutoOrder.size() - 1) {
      if (PathConverter.AutoOrder.get(AutoStage).equals("Grab Cone")) {
        System.out.println("Grab Cone");
        RobotMechanisms.DesiredState = "Grab";
        RobotMechanisms.openGrabber();
        if(RobotMechanisms.isAtDesiredState()) {
          if(Timer.get() > 0) { //If we are grabbing the cone, we don't want to open grabber, nor keep moving
            Tracking.centerOnCone();
          }
          if(Math.abs(Tracking.IntakeTargetOffsetV.getDouble(0)) <= 20) {//moved towards cone
            if(Timer.get() <= 0.0) {
              RobotMechanisms.closeGrabber();
              Timer.start();
            }
            if(Timer.get() >= 0.1) { //waited for grabber to close
              RobotMechanisms.DesiredState = "High";
              AutoStage++;
              Timer.stop();
              Timer.reset();
            }
          }
        }
      }
    }
    if (AutoStage <= PathConverter.AutoOrder.size() - 1) {
      if (PathConverter.AutoOrder.get(AutoStage).equals("Grab Cube")) {
        System.out.println("Grab Cube");
        RobotMechanisms.DesiredState = "Grab";
        RobotMechanisms.openGrabber();
        if(RobotMechanisms.isAtDesiredState()) {
          if(Timer.get() > 0) { //If we are grabbing the cone, we don't want to open grabber, nor keep moving
            Tracking.centerOnCube();
          }
          if(Math.abs(Tracking.IntakeTargetOffsetV.getDouble(0)) <= 20) {//moved towards cone
            if(Timer.get() <= 0.0) {
              RobotMechanisms.closeGrabber();
              Timer.start();
            }
            if(Timer.get() >= 0.1) { //waited for grabber to close
              RobotMechanisms.DesiredState = "High";
              AutoStage++;
              Timer.stop();
              Timer.reset();
            }
          }
        }
      }
    }
    if (AutoStage <= PathConverter.AutoOrder.size() - 1) {
      if (PathConverter.AutoOrder.get(AutoStage).equals("Place Cone")) {
        System.out.println("Place Cone");
        RobotMechanisms.DesiredState = "Place1";
        System.out.println("Arm Angle: " + RobotMechanisms.ArmAngle.getEncoder().getPosition());
        if(RobotMechanisms.isAtDesiredState()) {
          if(Timer.get() <= 0.0) {
            RobotMechanisms.openGrabber();
            Timer.start();
          }
          if(Timer.get() >= 0.2) { //wait for grabber to open
            RobotMechanisms.DesiredState = "Grab";
            AutoStage++;
            Timer.stop();
            Timer.reset();
          }
          // Tracking.centerOnPole();
          // if(Math.abs(Tracking.ArmTargetOffsetH.getDouble(0)) <= 20) {
          //   if(Timer.get() <= 0.0) {
          //     RobotMechanisms.openGrabber();
          //     Timer.start();
          //   }
          //   if(Timer.get() >= 0.2) { //waited for grabber to open
          //     RobotMechanisms.DesiredState = "Stowed";
          //     AutoStage++;
          //     Timer.stop();
          //     Timer.reset();
          //   }
        }
      }
    }
    if (AutoStage <= PathConverter.AutoOrder.size() - 1) {
      if (PathConverter.AutoOrder.get(AutoStage).equals("Place Cube")) {
        System.out.println("Place Cube");
        RobotMechanisms.DesiredState = "Place1";
        if(RobotMechanisms.isAtDesiredState()) {
          Tracking.centerOnPlatform();
          if(Math.abs(Tracking.ArmTargetOffsetH.getDouble(0)) <= 20) {
            RobotMechanisms.openGrabber();
            RobotMechanisms.DesiredState = "High";
            AutoStage++;
          }
        }
      }
    }
    if (AutoStage <= PathConverter.AutoOrder.size() - 1) {
      if (PathConverter.AutoOrder.get(AutoStage).equals("Charge")) {
        System.out.println("Charge");
        AutoStage++;
      }
    }
    
    RobotMechanisms.goToDesiredState();
  }
}
