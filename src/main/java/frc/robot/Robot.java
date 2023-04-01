// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.networktables.*;
import frc.PathConverter.PathConverter;
import frc.SwerveSubsystem.SwerveDrive;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.RobotMechanisms;
import frc.robot.subsystems.Tracking;



public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick LeftStick;
  private Joystick RightStick;

  private double RightStickX;
  private double RightStickY;
  private double RightStickTwist;
  private double[] MotorCurrents;
  private boolean ManualArmRead;

  public NetworkTableInstance Inst;
  public NetworkTable DriverStation;
  public NetworkTableEntry GyroAng;
  private ShuffleboardTab LiveWindow;

  private SimpleWidget FFGain;
  private SimpleWidget PGain;
  private SimpleWidget IGain;
  private SimpleWidget DGain;

  @Override
  public void robotInit() {
    // Instantiate an object for each class
    SwerveDrive.init();
    PathConverter.init();
    RobotMechanisms.init();
    Tracking.init();
    Autonomous.init();

    // Assign joysticks to the "LeftStick" and "RightStick" objects
    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);

    Inst = NetworkTableInstance.getDefault();
    ManualArmRead = false;

    LiveWindow = Shuffleboard.getTab("LiveWindow");
  
    FFGain = LiveWindow.add("FFGain", 0.000175);
    PGain = LiveWindow.add("PGain", 0.00001);
    IGain = LiveWindow.add("IGain", 0.0000004);
    DGain = LiveWindow.add("DGain", 0.0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    /**
    RobotMechanisms.ArmExtendPIDController.setFF(FFGain.getEntry().getDouble(0));
    RobotMechanisms.ArmExtendPIDController.setP(PGain.getEntry().getDouble(0));
    RobotMechanisms.ArmExtendPIDController.setI(IGain.getEntry().getDouble(0));
    RobotMechanisms.ArmExtendPIDController.setD(DGain.getEntry().getDouble(0));
    */
    
    //SwerveDrive.updatePIDValues(FFGain.getEntry().getDouble(0), PGain.getEntry().getDouble(0), IGain.getEntry().getDouble(0), DGain.getEntry().getDouble(0), 8.0, 0.01, 0.01);
  }
 
  @Override
  public void teleopInit() {
    RobotMechanisms.extendLimelight();
  }

  @Override
  public void teleopPeriodic() {
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d().unaryMinus();

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
      //Tracking.centerOnPole();
    }
    else if (RobotMechanisms.DesiredState != "Charge") {
      // Call swerveDrive() method, to do all the math and outputs for swerve drive
      SwerveDrive.calculateSpeedsAndAngles(Math.copySign(Math.pow(RightStickX, 2.0), RightStickX) * Constants.SwerveMaxVelocity, (-Math.copySign(Math.pow(RightStickY, 2.0), RightStickY) * Constants.SwerveMaxVelocity), (Math.copySign(Math.pow(RightStickTwist, 2.0), RightStickTwist) * Constants.SwerveMaxVelocity), (1 - ((RightStick.getZ() + 1) / 2)), (1 - ((LeftStick.getZ() + 1) / 2)));
      SwerveDrive.optimizeAndSetOutputs();
    }

    SmartDashboard.putNumber("Gyro", SwerveDrive.GyroRotation2d.getDegrees());

    MotorCurrents = new double[] {SwerveDrive.FrontLeft.Drive.getOutputCurrent(), SwerveDrive.FrontRight.Drive.getOutputCurrent(), SwerveDrive.BackLeft.Drive.getOutputCurrent(), SwerveDrive.BackRight.Drive.getOutputCurrent()};
    SmartDashboard.putNumberArray("RobotDrive Motors", MotorCurrents);
  
    if (RightStick.getRawButtonPressed(2) == true) {
      RobotMechanisms.reset();
    }
    if (LeftStick.getRawButtonPressed(5)) {
      RobotMechanisms.DesiredState = "Stowed";
      RobotMechanisms.ArmAngleMod = 0;
      RobotMechanisms.ArmExtendMod = 0;
    }
    else if (LeftStick.getRawButtonPressed(2)) {
      RobotMechanisms.DesiredState = "Grab";
      RobotMechanisms.ArmAngleMod = 0;
      RobotMechanisms.ArmExtendMod = 0;
    }
    else if (LeftStick.getRawButtonPressed(6)) {
      RobotMechanisms.DesiredState = "Place1";
      RobotMechanisms.ArmAngleMod = 0;
      RobotMechanisms.ArmExtendMod = 0;
    }
    else if (LeftStick.getRawButtonPressed(4)) {
      RobotMechanisms.DesiredState = "Place2";
      RobotMechanisms.ArmAngleMod = 0;
      RobotMechanisms.ArmExtendMod = 0;
    }
    else if (LeftStick.getRawButtonPressed(3)) {
      RobotMechanisms.DesiredState = "High";
      RobotMechanisms.ArmAngleMod = 0;
      RobotMechanisms.ArmExtendMod = 0;
    }
    // else if (RightStick.getRawButtonPressed(3)){
    //   RobotMechanisms.DesiredState = "Charge";
    //   RobotMechanisms.ArmAngleMod = 0;
    //   RobotMechanisms.ArmExtendMod = 0;
    // }
    else if (RightStick.getRawButtonPressed(5)) {
      SwerveDrive.FieldOrientedSwerveEnabled = !SwerveDrive.FieldOrientedSwerveEnabled;
      System.out.println("Field oriented swerve state updated to " + SwerveDrive.FieldOrientedSwerveEnabled);
    }

    if (LeftStick.getPOV() == 0 & !ManualArmRead) {
      ManualArmRead = true;
      RobotMechanisms.ArmAngleMod += 0.5;
    }
    if (LeftStick.getPOV() == 180 & !ManualArmRead) {
      ManualArmRead = true;
      RobotMechanisms.ArmAngleMod -= 0.5;
    }
    if (LeftStick.getPOV() == -1 & ManualArmRead) {
      ManualArmRead = false;
    }

    if (LeftStick.getRawButtonPressed(1)) {
      RobotMechanisms.grabObject();
    }
    if (RightStick.getRawButtonPressed(4)) {
      RobotMechanisms.extendLimelight();
    }
    if (RightStick.getRawButtonPressed(6)) {
      RobotMechanisms.retractLimelight();
    }

    RobotMechanisms.goToDesiredState();
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
    RobotMechanisms.extendLimelight();
    Autonomous.Timer.stop();
    Autonomous.Timer.reset();
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d(); 
    Autonomous.runAutonomous();
  }
}