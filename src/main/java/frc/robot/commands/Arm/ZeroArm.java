// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroArm extends CommandBase {
  public ZeroArm(Arm arm) {
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.hasBeenZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Arm.limitSwitch.get()) {
      Arm.hasBeenZeroed = true;
      Arm.armExtend.getEncoder().setPosition(0);
      Arm.armExtendPIDController.setReference(0, ControlType.kPosition);
    }
    else {
      Arm.armExtend.set(.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Arm.hasBeenZeroed;
  }
}
