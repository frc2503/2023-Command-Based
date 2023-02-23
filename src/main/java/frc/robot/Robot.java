// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.FileNotFoundException;

import javax.lang.model.util.ElementScanner14;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Tracking;



public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick LeftStick;
  private Joystick RightStick;
  private double RightStickX;
  private double RightStickY;
  private double RightStickTwist;
  private double[] MotorCurrents;
  private String[] AutoNames;
  private String PrevAuto;

  public SwerveDrive SwerveDrive;
  public Tracking Tracking;
  public Autonomous Autonomous;

  public NetworkTableInstance Inst;
  public NetworkTable DriverStation;
  public NetworkTableEntry GyroAng;
  public SendableChooser<String> AutoChooser;
  private ShuffleboardTab LiveWindow;
  private SimpleWidget FFGain;
  private SimpleWidget PGain;
  private SimpleWidget IGain;
  private SimpleWidget DGain;

  private CANSparkMax ArmAngle;
  private SparkMaxPIDController ArmAnglePIDController;
  private CANSparkMax ArmExtend;
  private SparkMaxPIDController ArmExtendPIDController;

  private Compressor Pump;
  private DoubleSolenoid Grabber;


  @Override
  public void robotInit() {
    Inst = NetworkTableInstance.getDefault();

    AutoChooser = new SendableChooser<String>();
    AutoNames = Filesystem.getDeployDirectory().toPath().resolve("output/paths").toFile().list();
    for (Integer Index = 0; Index <= AutoNames.length - 1; Index++) {
      AutoChooser.addOption(AutoNames[Index], AutoNames[Index]);
    }
    AutoChooser.setDefaultOption("BlueTest", "BlueTest");
    SmartDashboard.putData("AutoChooser", AutoChooser);

    LiveWindow = Shuffleboard.getTab("LiveWindow");
    /**
    FFGain = LiveWindow.add("FFGain", 0.0);
    PGain = LiveWindow.add("PGain", 0.02);
    IGain = LiveWindow.add("IGain", 0.0);
    DGain = LiveWindow.add("DGain", 0.0);
    */
    // Assign joysticks to the "LeftStick" and "RightStick" objects
    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);

    ArmAngle = new CANSparkMax(10, MotorType.kBrushless);
    ArmAnglePIDController = ArmAngle.getPIDController();
    ArmExtend = new CANSparkMax(11, MotorType.kBrushless);
    ArmExtendPIDController = ArmExtend.getPIDController();

    ArmAnglePIDController.setFeedbackDevice(ArmAngle.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmAnglePIDController.setOutputRange(-1, 1);
    ArmAnglePIDController.setFF(0);
    ArmAnglePIDController.setP(.05);
    ArmAnglePIDController.setI(0);
    ArmAnglePIDController.setD(0);
    ArmExtendPIDController.setFeedbackDevice(ArmExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmExtendPIDController.setOutputRange(-1, 1);
    ArmAnglePIDController.setFF(0);
    ArmAnglePIDController.setP(.06);
    ArmAnglePIDController.setI(0);
    ArmAnglePIDController.setD(0);

    Pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    Grabber.set(Value.kForward);

    // Instantiate an object for each class
    SwerveDrive = new SwerveDrive();
    Tracking = new Tracking(SwerveDrive);
    Autonomous = new Autonomous(SwerveDrive, Tracking);

    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d();

    // Call SwerveDrive methods, their descriptions are in the SwerveDrive.java file
    SwerveDrive.initMotorControllers(1, 5, 2, 6, 3, 7, 4, 8);
    SwerveDrive.setPID(0.000175, 0.000001, 0.0000004, 0.0, 8.0, 0.01, 0.01);
    SwerveDrive.initKinematicsAndOdometry();
    PrevAuto = AutoChooser.getSelected();
    Autonomous.AutoFile = AutoChooser.getSelected();
    if (AutoChooser.getSelected() != null) {
      System.out.println(Autonomous.AutoFile);
    try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
    }
    Pump.enableDigital();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (PrevAuto != AutoChooser.getSelected()) {
      Autonomous.AutoFile = AutoChooser.getSelected();
      System.out.println(Autonomous.AutoFile);
      try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
      PrevAuto = AutoChooser.getSelected();
    }
    /**
    ArmExtendPIDController.setFF(FFGain.getEntry().getDouble(0));
    ArmExtendPIDController.setP(PGain.getEntry().getDouble(0));
    ArmExtendPIDController.setI(IGain.getEntry().getDouble(0));
    ArmExtendPIDController.setD(DGain.getEntry().getDouble(0));
    */
  }
 
  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d().unaryMinus();

    SwerveDrive.FrontLeft.DriveEncoder.getVelocity();

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

    if (LeftStick.getRawButton(2)) {
      Tracking.centerOnPole();
    }
    else {
      // Call swerveDrive() method, to do all the math and outputs for swerve drive
      SwerveDrive.swerveDrive(Math.pow(RightStickX, 3) * 2, (Math.pow(RightStickY, 3) * -2), (Math.pow(RightStickTwist, 3) * 2.5), (1 - ((RightStick.getZ() + 1) / 2)), (1 - ((LeftStick.getZ() + 1) / 2)));
      SwerveDrive.setVariablesAndOptimize();
      SwerveDrive.setSwerveOutputs();
    }

    SmartDashboard.putNumber("Gyro", SwerveDrive.GyroRotation2d.getDegrees());

    MotorCurrents = new double[] {SwerveDrive.FrontLeft.Drive.getOutputCurrent(), SwerveDrive.FrontRight.Drive.getOutputCurrent(), SwerveDrive.BackLeft.Drive.getOutputCurrent(), SwerveDrive.BackRight.Drive.getOutputCurrent()};
    SmartDashboard.putNumberArray("RobotDrive Motors", MotorCurrents);
  
    if (RightStick.getRawButtonPressed(2) == true) {
      SwerveDrive.Gyro.reset();
      ArmAngle.getEncoder().setPosition(0);
      ArmExtend.getEncoder().setPosition(0);
    }
    if (LeftStick.getRawButtonPressed(4)) {
      ArmAnglePIDController.setReference(-22, ControlType.kPosition);
    }
    else if (LeftStick.getRawButtonPressed(6)) {
      ArmAnglePIDController.setReference(0, ControlType.kPosition);
    }
    if (LeftStick.getRawButtonPressed(5)) {
      ArmExtendPIDController.setReference(0, ControlType.kPosition);
    }
    else if (LeftStick.getRawButtonPressed(3)) {
      ArmExtendPIDController.setReference(56, ControlType.kPosition);
    }
    if (LeftStick.getRawButtonPressed(1)) {
      Grabber.toggle();
    }
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d(); 
    Autonomous.runAutonomous();
  }
}