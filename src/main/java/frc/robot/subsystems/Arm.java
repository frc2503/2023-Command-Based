// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private static CANSparkMax armAngle = new CANSparkMax(Constants.armAngleCANID, MotorType.kBrushless);
  private static SparkMaxPIDController armAnglePIDController = armAngle.getPIDController();
  private static CANSparkMax armExtend = new CANSparkMax(Constants.armExtendCANID, MotorType.kBrushless);
  static SparkMaxPIDController armExtendPIDController = armExtend.getPIDController();

  private static Compressor pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private static DoubleSolenoid grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private static DoubleSolenoid limelightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  private static DigitalInput limitSwitch = new DigitalInput(0);
  public static boolean hasBeenZeroed = false;

  public static double armAngleMod = 0;
  public static double armExtendMod = 0;

  public Arm() {
    armAnglePIDController.setFeedbackDevice(armAngle.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    armAnglePIDController.setOutputRange(-1, 1);
    armAnglePIDController.setFF(0);
    armAnglePIDController.setP(.06);
    armAnglePIDController.setI(0);
    armAnglePIDController.setD(0);
    armAnglePIDController.setSmartMotionMinOutputVelocity(0, 0);
    armAnglePIDController.setSmartMotionMaxVelocity(120, 0);
    armAnglePIDController.setSmartMotionMaxAccel(50, 0);

    armExtendPIDController.setFeedbackDevice(armExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    armExtendPIDController.setOutputRange(-1, 1);
    armExtendPIDController.setFF(0);
    armExtendPIDController.setP(.06);
    armExtendPIDController.setI(0);
    armExtendPIDController.setD(0);
    armExtendPIDController.setSmartMotionMinOutputVelocity(0, 0);
    armExtendPIDController.setSmartMotionMaxVelocity(120, 0);

    pump.enableDigital();
    grabber.set(Value.kForward);
    limelightPiston.set(Value.kForward);
  }

  public static boolean zeroArm() {
    if (!limitSwitch.get()) {
      hasBeenZeroed = true;
      armExtend.getEncoder().setPosition(0);
      armExtendPIDController.setReference(0, ControlType.kPosition);
    }
    else {
      armExtend.set(.2);
    }
    return hasBeenZeroed;
  }

  public static boolean stow() {
    if (Math.abs(armExtend.getEncoder().getPosition()) > .6) {
      Arm.armExtendPIDController.setReference(0, ControlType.kPosition);
    } else {
      Arm.armAnglePIDController.setReference(0, ControlType.kPosition);
      Arm.armExtendPIDController.setReference(0, ControlType.kPosition);
    }
    return Math.abs(armExtend.getEncoder().getPosition()) <= .6 && Math.abs(armAngle.getEncoder().getPosition()) <= 1;
  }

  public static boolean grab() {
    if (armAngle.getEncoder().getPosition() < -7 & armExtend.getEncoder().getPosition() < -43) {
      armExtendPIDController.setReference(-52, ControlType.kPosition);
    } else if (armAngle.getEncoder().getPosition() > -3 & armExtend.getEncoder().getPosition() < -.6) {
      armExtendPIDController.setReference(0, ControlType.kPosition);
    } else if (armAngle.getEncoder().getPosition() > -4) {
      armAnglePIDController.setReference(-6.5 - armAngleMod, ControlType.kPosition);
    } else {
      armAnglePIDController.setReference(-6.5 - armAngleMod, ControlType.kPosition);
      armExtendPIDController.setReference(-52, ControlType.kPosition);
    }
    return Math.abs(Math.abs(armAngle.getEncoder().getPosition()) - 6.5) <= 1 &&
    Math.abs(Math.abs(armExtend.getEncoder().getPosition()) - 41) <= 1;
  }

  public static boolean placeLow() {
    if (armAngle.getEncoder().getPosition() > -6) {
      armAnglePIDController.setReference(-19 - armAngleMod, ControlType.kPosition);
      armExtendPIDController.setReference(0, ControlType.kPosition);
    } else {
      armAnglePIDController.setReference(-19 - armAngleMod, ControlType.kPosition);
      armExtendPIDController.setReference(-37, ControlType.kPosition);
    }
    return Math.abs(Math.abs(armAngle.getEncoder().getPosition()) - 17) <= 1
    && Math.abs(Math.abs(armExtend.getEncoder().getPosition()) - 37) <= 1;
  }

  public static boolean placeHigh() {
    if (armAngle.getEncoder().getPosition() > -6) {
      armAnglePIDController.setReference(-23 - armAngleMod, ControlType.kPosition);
      armExtendPIDController.setReference(0, ControlType.kPosition);
    } else {
      armAnglePIDController.setReference(-23 - armAngleMod, ControlType.kPosition);
      armExtendPIDController.setReference(-85, ControlType.kPosition);
    }
    return Math.abs(Math.abs(armAngle.getEncoder().getPosition()) - 21) <= 1
    && Math.abs(Math.abs(armExtend.getEncoder().getPosition()) - 85) <= 1;
  }

  /**
   * Command the robot to toggle the grabber.
   */
  public static void toggleGrabber() {
    grabber.toggle();
  }

  /**
   * Command the robot to close the grabber.
   */
  public static void closeGrabber() {
    if (grabber.get() != Value.kForward) {
      grabber.set(Value.kForward);
    }
  }

  /**
   * Command the robot to open the grabber.
   */
  public static void openGrabber() {
    if (grabber.get() != Value.kReverse) {
      grabber.set(Value.kReverse);
    }
  }

  /**
   * Command the robot to extend the Limelight piston.
   */
  public static void extendLimelight() {
    limelightPiston.set(Value.kReverse);
  }

  /**
   * Command the robot to retract the Limelight piston.
   */
  public static void retractLimelight() {
    limelightPiston.set(Value.kForward);
  }
}
