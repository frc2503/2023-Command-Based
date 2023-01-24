// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.FileNotFoundException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick LeftStick;
  private Joystick RightStick;
  private Double RightStickX;
  private Double RightStickY;
  private Double RightStickTwist;
  private Double[] MotorCurrents;
  public SwerveDrive SwerveDrive;
  public Autonomous Autonomous;

  public NetworkTableInstance Inst;
  public NetworkTable DriverStation;
  public NetworkTableEntry GyroAng;
  public SendableChooser<String> AutoChooser;

  private CANSparkMax IntakeBottom;
  private CANSparkMax IntakeTop;

  @Override
  public void robotInit() {
    // Start getting video from USB camera
    CameraServer.startAutomaticCapture();
    
    Inst = NetworkTableInstance.getDefault();

    AutoChooser = new SendableChooser<String>();
    AutoChooser.addOption("Red1Charger", "Red1Charger");
    SmartDashboard.putData("AutoChooser", AutoChooser);
    
    // Assign joysticks to the "LeftStick" and "RightStick" objects
    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);

    IntakeBottom = new CANSparkMax(9, MotorType.kBrushless);
    IntakeTop = new CANSparkMax(10, MotorType.kBrushless);

    // Create an object for the SwerveDrive class
    SwerveDrive = new SwerveDrive();

    // Call SwerveDrive methods, their descriptions are in the SwerveDrive.java file
    SwerveDrive.initMotorControllers(1, 5, 2, 6, 3, 7, 4, 8);
    SwerveDrive.setPID(0.000175, 0.0000007, 0.0000001, 0.0, 8.0, 0.01, 0.01);
    SwerveDrive.initKinematicsAndOdometry();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    // Assign stick inputs to variables, to prevent discrepancies
    RightStickX = RightStick.getX();
    RightStickY = RightStick.getY();
    RightStickTwist = RightStick.getRawAxis(3);

    // Create deadzones on the joysticks, to prevent stick drift
    if (Math.abs(RightStickX) < 0.1) {
      RightStickX = 0.0;
    }
    if (Math.abs(RightStickY) < 0.1) {
      RightStickY = 0.0;
    }
    if (Math.abs(RightStickTwist) < 0.2) {
      RightStickTwist = 0.0;
    }

    // Call swerveDrive() method, to do all the math and outputs for swerve drive
    SwerveDrive.swerveDrive(RightStickX * 2, (RightStickY * -2), (RightStickTwist * 2.5), (1 - ((RightStick.getZ() + 1) / 2)), (1 - ((LeftStick.getZ() + 1) / 2)));
    SwerveDrive.setVariablesAndOptimize();
    SwerveDrive.setSwerveOutputs();

    SmartDashboard.putNumber("Gyro", SwerveDrive.GyroRotation2d.unaryMinus().getDegrees());

    MotorCurrents = new Double[] {SwerveDrive.FrontLeft.Drive.getOutputCurrent(), SwerveDrive.FrontRight.Drive.getOutputCurrent(), SwerveDrive.BackLeft.Drive.getOutputCurrent(), SwerveDrive.BackRight.Drive.getOutputCurrent()};
    SmartDashboard.putNumberArray("RobotDrive Motors", MotorCurrents);
  
    if (RightStick.getRawButton(2) == true) {
      SwerveDrive.Gyro.reset();
    }
    if (RightStick.getRawButton(3)) {
      IntakeBottom.set(.3);
      IntakeTop.set(-.3);
    }
    else {
      IntakeBottom.set(0);
      IntakeTop.set(0);
    }
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
    try {
      Autonomous.initTrajectory();
    }
    catch (FileNotFoundException Error) {
      System.out.println("AUTO NOT FOUND");
    }
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){ 
    Autonomous.runAutonomous();
  }
}