// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.swervesubsystem.SwerveDrive;

public class ShuffleBoard extends SubsystemBase {
  private double[] motorCurrents = new double[] { SwerveDrive.frontLeft.drive.getOutputCurrent(),
    SwerveDrive.frontRight.drive.getOutputCurrent(), SwerveDrive.backLeft.drive.getOutputCurrent(),
    SwerveDrive.backRight.drive.getOutputCurrent()};
  private ShuffleboardTab liveWindow = Shuffleboard.getTab("liveWindow");

  private SimpleWidget FFGain = liveWindow.add("FFGain", Constants.driveFeedForward);
  private SimpleWidget PGain = liveWindow.add("PGain", Constants.driveProportional);
  private SimpleWidget IGain = liveWindow.add("IGain", Constants.driveIntegral);
  private SimpleWidget DGain = liveWindow.add("DGain", Constants.driveDerivative);

  @Override
  public void periodic() {
    /*
    Arm.armExtendPIDController.setFF(FFGain.getEntry().getDouble(0));
    Arm.armExtendPIDController.setP(PGain.getEntry().getDouble(0));
    Arm.armExtendPIDController.setI(IGain.getEntry().getDouble(0));
    Arm.armExtendPIDController.setD(DGain.getEntry().getDouble(0));
    
    SwerveDrive.updatePIDValues(FFGain.getEntry().getDouble(0), PGain.getEntry().getDouble(0),
        IGain.getEntry().getDouble(0), DGain.getEntry().getDouble(0), 0, 8.0, 0.01, 0.01);
    */

    SmartDashboard.putNumber("Gyro", SwerveDrive.gyroRotation2d.getDegrees());

    motorCurrents[0] = SwerveDrive.frontLeft.drive.getOutputCurrent();
    motorCurrents[1] = SwerveDrive.frontRight.drive.getOutputCurrent();
    motorCurrents[2] = SwerveDrive.backLeft.drive.getOutputCurrent();
    motorCurrents[3] = SwerveDrive.backRight.drive.getOutputCurrent();
    SmartDashboard.putNumberArray("RobotDrive Motors", motorCurrents);
  }
}
