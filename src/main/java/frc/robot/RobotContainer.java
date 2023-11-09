// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drivetrain.Drive;
import frc.robot.commands.RobotMechanisms.Grab;
import frc.robot.commands.RobotMechanisms.PlaceHigh;
import frc.robot.commands.RobotMechanisms.PlaceLow;
import frc.robot.commands.RobotMechanisms.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Tracking;
import frc.swervesubsystem.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final SwerveDrive SWERVE_DRIVE = new SwerveDrive();
  private static final Arm ARM = new Arm();
  private static final Tracking TRACKING = new Tracking();
  
  private Joystick rightStick = new Joystick(Constants.rightStickPort);
  private Joystick leftStick = new Joystick(Constants.leftStickPort);

  private final JoystickButton rightStickTrigger = new JoystickButton(rightStick, 1);
  private final JoystickButton rightStickButton2 = new JoystickButton(rightStick, 2);
  private final JoystickButton rightStickButton3 = new JoystickButton(rightStick, 3);
  private final JoystickButton rightStickButton4 = new JoystickButton(rightStick, 4);
  private final JoystickButton rightStickButton5 = new JoystickButton(rightStick, 5);
  private final JoystickButton rightStickButton6 = new JoystickButton(rightStick, 6);

  private final JoystickButton leftStickTrigger = new JoystickButton(rightStick, 1);
  private final JoystickButton leftStickButton2 = new JoystickButton(rightStick, 2);
  private final JoystickButton leftStickButton3 = new JoystickButton(rightStick, 3);
  private final JoystickButton leftStickButton4 = new JoystickButton(rightStick, 4);
  private final JoystickButton leftStickButton5 = new JoystickButton(rightStick, 5);
  private final JoystickButton leftStickButton6 = new JoystickButton(rightStick, 6);

  private final SendableChooser<Command> autoChooser;
  
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Stow", new Stow(ARM));
    NamedCommands.registerCommand("Grab", new Grab(ARM));
    NamedCommands.registerCommand("PlaceLow", new PlaceLow(ARM));
    NamedCommands.registerCommand("PlaceHigh", new PlaceHigh(ARM));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    SWERVE_DRIVE.setDefaultCommand(new Drive(SWERVE_DRIVE, () -> rightStick.getX(),
    () -> rightStick.getY(), () -> rightStick.getRawAxis(3),
    () -> rightStick.getZ(), () -> leftStick.getZ(), rightStickButton5));

    rightStickButton2.onTrue(new InstantCommand(SwerveDrive::resetGyro));
    rightStickButton4.onTrue(new InstantCommand(Arm::extendLimelight));
    rightStickButton6.onTrue(new InstantCommand(Arm::retractLimelight));
    leftStickTrigger.onTrue(new InstantCommand(Arm::toggleGrabber));
    leftStickButton2.onTrue(new Grab(ARM));
    leftStickButton4.onTrue(new PlaceHigh(ARM));
    leftStickButton5.onTrue(new Stow(ARM));
    leftStickButton6.onTrue(new PlaceLow(ARM));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}