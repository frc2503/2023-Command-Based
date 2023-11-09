// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.swervesubsystem.SwerveDrive;

public class Drive extends CommandBase {
  private final DoubleSupplier xTranslationSupplier;
  private final DoubleSupplier yTranslationSupplier;
  private final DoubleSupplier rotationSupplier;
  private final DoubleSupplier translationModSupplier;
  private final DoubleSupplier rotationModSupplier;
  private final BooleanSupplier isFieldOrientedSupplier;

  private double x;
  private double y;
  private double rot;
  private double translationMod;
  private double rotationMod;
  private boolean isFieldOriented;

  public Drive(SwerveDrive swerveDrive, DoubleSupplier xTranslationSupplier,
      DoubleSupplier yTranslationSupplier, DoubleSupplier rotationSupplier,
      DoubleSupplier translationModSupplier, DoubleSupplier rotationModSupplier,
      BooleanSupplier isFieldOrientedSupplier) {
    addRequirements(swerveDrive);
    this.xTranslationSupplier = xTranslationSupplier;
    this.yTranslationSupplier = yTranslationSupplier;
    this.rotationSupplier = rotationSupplier;
    this.translationModSupplier = translationModSupplier;
    this.rotationModSupplier = rotationModSupplier;
    this.isFieldOrientedSupplier = isFieldOrientedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = xTranslationSupplier.getAsDouble();
    y = yTranslationSupplier.getAsDouble();
    rot = rotationSupplier.getAsDouble();
    translationMod = translationModSupplier.getAsDouble();
    rotationMod = rotationModSupplier.getAsDouble();
    isFieldOriented = isFieldOrientedSupplier.getAsBoolean();

    if (Math.abs(x) < 0.1) {
      x = 0.0;
    }
    if (Math.abs(y) < 0.1) {
      y = 0.0;
    }
    if (Math.abs(rot) < 0.2) {
      rot = 0.0;
    }

    x = Math.copySign(Math.pow(x, 2.0), x) * Constants.swerveMaxVelocity;
    y = -Math.copySign(Math.pow(y, 2.0), y) * Constants.swerveMaxVelocity;
    rot = Math.copySign(Math.pow(rot, 2.0), rot) * Constants.swerveMaxVelocity;
    translationMod = (1 - ((translationMod + 1) / 2));
    rotationMod = (1 - ((rotationMod + 1) / 2));
    isFieldOriented = !isFieldOriented;

    SwerveDrive.calculateSpeedsAndAngles(x, y, rot, translationMod, rotationMod, isFieldOriented);
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
    return false;
  }
}
