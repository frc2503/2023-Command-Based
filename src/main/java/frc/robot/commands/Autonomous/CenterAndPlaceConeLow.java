// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.Low;
import frc.robot.commands.Tracking.CenterPole;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Tracking;
import frc.swervesubsystem.SwerveDrive;

public class CenterAndPlaceConeLow extends SequentialCommandGroup {
  public CenterAndPlaceConeLow(SwerveDrive swerveDrive, Arm arm, Tracking tracking) {
    addRequirements(swerveDrive, tracking);
    addCommands(new Low(arm), new CenterPole(swerveDrive, tracking), new InstantCommand(Arm::openGrabber));
  }
}
