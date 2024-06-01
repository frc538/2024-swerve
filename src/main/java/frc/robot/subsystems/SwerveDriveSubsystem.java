// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModule mFrontLeft = new SwerveModule(
      Constants.SparkMaxCANID.kFrontLeftDrive,
      Constants.SparkMaxCANID.kFrontLeftTurn,
      Constants.DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule mFrontRight = new SwerveModule(
      Constants.SparkMaxCANID.kFrontRightDrive,
      Constants.SparkMaxCANID.kFrontRightTurn,
      Constants.DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule mBackLeft = new SwerveModule(
      Constants.SparkMaxCANID.kRearLeftDrive,
      Constants.SparkMaxCANID.kRearLefttTurn,
      Constants.DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule mBackRight = new SwerveModule(
      Constants.SparkMaxCANID.kRearRightDrive,
      Constants.SparkMaxCANID.kRearRightTurn,
      Constants.DriveConstants.kBackRightChassisAngularOffset);

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem() {
  }

  public void setX() {
    mFrontLeft.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    mFrontRight.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackLeft.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackRight.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public Command setXCommand() {
    return Commands.run(() -> setX(), this);
  }

  public void drive(double forwardSpeed, double leftSpeed, double rotation){
    double xSpeedDelivered = forwardSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = leftSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double rotationDelivered = rotation * Constants.DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(
      new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    mFrontLeft.setmDesiredState(swerveModuleStates[0]);
    mFrontRight.setmDesiredState(swerveModuleStates[1]);
    mBackLeft.setmDesiredState(swerveModuleStates[2]);
    mBackRight.setmDesiredState(swerveModuleStates[3]);
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
