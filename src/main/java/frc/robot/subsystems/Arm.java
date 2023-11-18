// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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
  public static CANSparkMax armAngle = new CANSparkMax(Constants.armAngleCANID, MotorType.kBrushless);
  public static SparkMaxPIDController armAnglePIDController = armAngle.getPIDController();
  public static CANSparkMax armExtend = new CANSparkMax(Constants.armExtendCANID, MotorType.kBrushless);
  public static SparkMaxPIDController armExtendPIDController = armExtend.getPIDController();

  private static Compressor pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private static DoubleSolenoid grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private static DoubleSolenoid limelightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  public static DigitalInput limitSwitch = new DigitalInput(0);
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
