// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Grab extends CommandBase {
  boolean isFinished = false;
  
  public Grab(Arm arm) {
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.armAngleMod = 0;
    Arm.armExtendMod = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Arm.hasBeenZeroed) {
      if (Arm.armAngle.getEncoder().getPosition() < -7 & Arm.armExtend.getEncoder().getPosition() < -43) {
        Arm.armExtendPIDController.setReference(-52, ControlType.kPosition);
      } else if (Arm.armAngle.getEncoder().getPosition() > -3 & Arm.armExtend.getEncoder().getPosition() < -.6) {
        Arm.armExtendPIDController.setReference(0, ControlType.kPosition);
      } else if (Arm.armAngle.getEncoder().getPosition() > -4) {
        Arm.armAnglePIDController.setReference(-6.5 - Arm.armAngleMod, ControlType.kPosition);
      } else {
        Arm.armAnglePIDController.setReference(-6.5 - Arm.armAngleMod, ControlType.kPosition);
        Arm.armExtendPIDController.setReference(-52, ControlType.kPosition);
      }
      isFinished = Math.abs(Math.abs(Arm.armAngle.getEncoder().getPosition()) - 6.5) <= 1 &&
      Math.abs(Math.abs(Arm.armExtend.getEncoder().getPosition()) - 41) <= 1;
    } else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}