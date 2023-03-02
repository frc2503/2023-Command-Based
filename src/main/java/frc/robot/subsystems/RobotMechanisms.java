// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotMechanisms extends SubsystemBase {
  private SwerveDrive Swerve;
  
  public CANSparkMax ArmAngle;
  private SparkMaxPIDController ArmAnglePIDController;
  public CANSparkMax ArmExtend;
  private SparkMaxPIDController ArmExtendPIDController;

  private Constraints ChargeConstraints;
  private ProfiledPIDController ChargePIDController;

  private Compressor Pump;
  private DoubleSolenoid Grabber;
  private DoubleSolenoid LimelightPiston;

  private DigitalInput LimitSwitch;
  private boolean HasBeenZeroed;

  public String DesiredState;
  private double AngleToAdjust;

  public RobotMechanisms(SwerveDrive SwerveDrive) {
    Swerve = SwerveDrive;

    DesiredState = "";

    ArmAngle = new CANSparkMax(10, MotorType.kBrushless);
    ArmAnglePIDController = ArmAngle.getPIDController();
    ArmExtend = new CANSparkMax(11, MotorType.kBrushless);
    ArmExtendPIDController = ArmExtend.getPIDController();

    ArmAnglePIDController.setFeedbackDevice(ArmAngle.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmAnglePIDController.setOutputRange(-1, 1);
    ArmAnglePIDController.setFF(0);
    ArmAnglePIDController.setP(.06);
    ArmAnglePIDController.setI(0);
    ArmAnglePIDController.setD(0);
    ArmAnglePIDController.setSmartMotionMinOutputVelocity(0, 0);
    ArmAnglePIDController.setSmartMotionMaxVelocity(120, 0);
    ArmAnglePIDController.setSmartMotionMaxAccel(50, 0);

    ArmExtendPIDController.setFeedbackDevice(ArmExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmExtendPIDController.setOutputRange(-1, 1);
    ArmExtendPIDController.setFF(0);
    ArmExtendPIDController.setP(.06);
    ArmExtendPIDController.setI(0);
    ArmExtendPIDController.setD(0);
    ArmExtendPIDController.setSmartMotionMinOutputVelocity(0, 0);
    ArmExtendPIDController.setSmartMotionMaxVelocity(120, 0);

    ChargeConstraints = new Constraints(1, 2);
    ChargePIDController = new ProfiledPIDController(1, 0, 0, ChargeConstraints);

    Pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Pump.enableDigital();
    Grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    Grabber.set(Value.kForward);
    LimelightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    LimelightPiston.set(Value.kForward);

    LimitSwitch = new DigitalInput(0);
    HasBeenZeroed = false;
  }

  public void goToDesiredState() {
    if (HasBeenZeroed) {
      if (DesiredState == "Stowed") {
        closeGrabber();
        if (Math.abs(ArmExtend.getEncoder().getPosition()) > .6) {
          ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
        }
        else {
          ArmAnglePIDController.setReference(0, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
        }
      }

      if (DesiredState == "Grab") {
        if (ArmAngle.getEncoder().getPosition() < -6 & ArmExtend.getEncoder().getPosition() < -40) {
          ArmExtendPIDController.setReference(-39, ControlType.kSmartMotion);
        }
        else if (ArmAngle.getEncoder().getPosition() > -3 & ArmExtend.getEncoder().getPosition() < -.6) {
          ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
        }
        else if (ArmAngle.getEncoder().getPosition() > -4) {
          ArmAnglePIDController.setReference(-5.5, ControlType.kSmartMotion);
        }
        else {
          ArmAnglePIDController.setReference(-5.5, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(-41, ControlType.kSmartMotion);
        }
      }

      if (DesiredState == "Place1") {
        if (ArmAngle.getEncoder().getPosition() > -6) {
          ArmAnglePIDController.setReference(-21, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
        }
        else {
          ArmAnglePIDController.setReference(-21, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(-35, ControlType.kSmartMotion);
        }
      }

      if (DesiredState == "Place2") {
        if (ArmAngle.getEncoder().getPosition() > -6) {
          ArmAnglePIDController.setReference(-24, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
        }
        else {
          ArmAnglePIDController.setReference(-24, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(-85, ControlType.kSmartMotion);
        }
      }
      if (DesiredState == "High") {
        if (ArmAngle.getEncoder().getPosition() > -6) {
          ArmAnglePIDController.setReference(-24, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
        }
        else {
          ArmAnglePIDController.setReference(-24, ControlType.kSmartMotion);
          ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
        }
      }
      if (DesiredState == "Charge") {
        if (Math.abs(Swerve.Gyro.getRoll()) > Math.abs(Swerve.Gyro.getPitch())) {
          AngleToAdjust = Swerve.Gyro.getRoll();
        }
        else {
          AngleToAdjust = Swerve.Gyro.getPitch();
        }
        System.out.println(AngleToAdjust);
        Swerve.swerveDrive(0, ChargePIDController.calculate(AngleToAdjust, 0)/300, 0, 1, 1);
        Swerve.setVariablesAndOptimize();
        Swerve.setSwerveOutputs();
        System.out.println("Output:" + ChargePIDController.calculate(AngleToAdjust * 20, 0));
      }
    }
    else {
      if (!LimitSwitch.get()) {
        HasBeenZeroed = true;
        ArmExtend.getEncoder().setPosition(0);
        ArmExtendPIDController.setReference(0, ControlType.kSmartMotion);
      }
      else {
        ArmExtend.set(.2);
      }
    }

    if (!LimitSwitch.get()) {
      ArmExtend.getEncoder().setPosition(0);
    }
  }

  public boolean isAtDesiredState() {
    if(DesiredState == "Stowed" & ArmAngle.getEncoder().getPosition() >= -1 & ArmExtend.getEncoder().getPosition() <= 1) return true;
    if(DesiredState == "Grab" & Math.abs(ArmAngle.getEncoder().getPosition() + 6) <= 1 & Math.abs(ArmExtend.getEncoder().getPosition() - 39) <= 1) return true;
    if(DesiredState == "Place1" & Math.abs(ArmAngle.getEncoder().getPosition() + 23) <= 1 & Math.abs(ArmExtend.getEncoder().getPosition() - 43.5) <= 1) return true;
    if(DesiredState == "Place2" & Math.abs(ArmAngle.getEncoder().getPosition() + 23) <= 1 & Math.abs(ArmExtend.getEncoder().getPosition() - 56) <= 1) return true;
    if(DesiredState == "High" & ArmAngle.getEncoder().getPosition() <= -25 & ArmExtend.getEncoder().getPosition() >= 56) return true;
    else return false;
  }

  public void grabObject() {
    Grabber.toggle();
  }
  public void closeGrabber() {
    Grabber.set(Value.kForward);
  }
  public void openGrabber() {
    Grabber.set(Value.kReverse);
  }
  public void extendLimelight() {
    LimelightPiston.set(Value.kReverse);
  }
  public void retractLimelight() {
    LimelightPiston.set(Value.kForward);
  }

  public void reset() {
    Swerve.Gyro.reset();
    ArmAngle.getEncoder().setPosition(0);
    ArmExtend.getEncoder().setPosition(0);
  }
}
