// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.PathConverter.PathConverter;
import frc.SwerveSubsystem.SwerveDrive;
import frc.SwerveSubsystem.Wheel;
import frc.robot.subsystems.RobotMechanisms;
import frc.robot.subsystems.Tracking;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Wheel WheelSubsystem = new Wheel(0, 0);
  private final SwerveDrive SwerveDriveSubsystem = new SwerveDrive();
  private final RobotMechanisms RobotMechanismsSubsystem = new RobotMechanisms();
  private final Tracking TrackingSubsystem = new Tracking();
  private final PathConverter AutonomousSubsystem = new PathConverter();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}
}
