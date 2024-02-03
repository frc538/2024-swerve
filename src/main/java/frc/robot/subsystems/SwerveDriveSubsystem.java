// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem() {
  }

  public void robotCentricDrive(double forwardSpeed, double rightSpeed, double counterclockwiseRotation) {
    SmartDashboard.putNumber("Forward", forwardSpeed);
    SmartDashboard.putNumber("Right", rightSpeed);
    SmartDashboard.putNumber("CounterClockWise", counterclockwiseRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
