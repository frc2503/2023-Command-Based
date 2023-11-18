// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command autonomousCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
 
  @Override
  public void teleopInit() {
    Arm.extendLimelight();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  //Autonomous right away
  @Override
  public void autonomousInit() {
    Arm.extendLimelight();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }
}