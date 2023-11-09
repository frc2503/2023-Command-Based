// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tracking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tracking;
import frc.swervesubsystem.SwerveDrive;

public class CenterPlatform extends CommandBase {
  boolean isFinished = false;

  public CenterPlatform(SwerveDrive swerveDrive, Tracking tracking) {
    addRequirements(swerveDrive, tracking);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Tracking.selectPlatformTargeting();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Tracking.armTargetOffsetH.getDouble(0) >= 1) {
      SwerveDrive.calculateSpeedsAndAngles(-Tracking.PID.calculate(Tracking.armTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0, false);
    }
    else {
      SwerveDrive.calculateSpeedsAndAngles(-Tracking.PID.calculate(Tracking.armTargetOffsetH.getDouble(0), 0.0), -Tracking.PID.calculate(Tracking.armTargetOffsetV.getDouble(0), 0.0), 0.0, .1, 0, false);
      if (Tracking.armTargetOffsetV.getDouble(0) < 1) {
        isFinished = true;
      }
    }
    SwerveDrive.optimizeAndSetOutputs();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
