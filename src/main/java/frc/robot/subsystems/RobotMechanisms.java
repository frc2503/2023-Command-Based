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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotMechanisms extends SubsystemBase {
  private SwerveDrive Swerve;
  
  public CANSparkMax ArmAngle;
  private SparkMaxPIDController ArmAnglePIDController;
  public CANSparkMax ArmExtend;
  private SparkMaxPIDController ArmExtendPIDController;

  private Compressor Pump;
  private DoubleSolenoid Grabber;

  public String DesiredState;

  public RobotMechanisms(SwerveDrive SwerveDrive) {
    Swerve = SwerveDrive;

    DesiredState = "Stowed";

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

    ArmExtendPIDController.setFeedbackDevice(ArmExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmExtendPIDController.setOutputRange(-1, 1);
    ArmExtendPIDController.setFF(0);
    ArmExtendPIDController.setP(.06);
    ArmExtendPIDController.setI(0);
    ArmExtendPIDController.setD(0);

    Pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Pump.enableDigital();
    Grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    Grabber.set(Value.kForward);
  }

  public void goToDesiredState() {
    if (DesiredState == "Stowed") {
      if (Math.abs(ArmExtend.getEncoder().getPosition()) > 20) {
        ArmExtendPIDController.setReference(0, ControlType.kPosition);
      }
      else {
        ArmAnglePIDController.setReference(0, ControlType.kPosition);
        ArmExtendPIDController.setReference(0, ControlType.kPosition);
      }
    }

    if (DesiredState == "Grab") {
      if (ArmAngle.getEncoder().getPosition() > -5.5) {
        ArmAnglePIDController.setReference(-6, ControlType.kPosition);
      }
      else {
        ArmAnglePIDController.setReference(-6, ControlType.kPosition);
        ArmExtendPIDController.setReference(39, ControlType.kPosition);
      }
    }

    if (DesiredState == "Place1") {
      if (ArmAngle.getEncoder().getPosition() > -5.5) {
        ArmAnglePIDController.setReference(-23, ControlType.kPosition);
      }
      else {
        ArmAnglePIDController.setReference(-23, ControlType.kPosition);
        ArmExtendPIDController.setReference(43.5, ControlType.kPosition);
      }
    }

    if (DesiredState == "Place2") {
      if (ArmAngle.getEncoder().getPosition() > -5.5) {
        ArmAnglePIDController.setReference(-23, ControlType.kPosition);
      }
      else {
        ArmAnglePIDController.setReference(-23, ControlType.kPosition);
        ArmExtendPIDController.setReference(56, ControlType.kPosition);
      }
    }
    if (DesiredState == "High") {
      if (ArmAngle.getEncoder().getPosition() > -5.5) {
        ArmAnglePIDController.setReference(-25, ControlType.kPosition);
      }
      else {
        ArmAnglePIDController.setReference(-25, ControlType.kPosition);
        ArmExtendPIDController.setReference(56, ControlType.kPosition);
      }
    }
  }

  public void grabObject() {
    Grabber.toggle();
  }

  public void reset() {
    Swerve.Gyro.reset();
    ArmAngle.getEncoder().setPosition(0);
    ArmExtend.getEncoder().setPosition(0);
  }
}
