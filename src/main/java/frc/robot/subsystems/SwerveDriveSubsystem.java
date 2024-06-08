// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.node.DoubleNode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveUtils;

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

  private double mPreviousTime = WPIUtilJNI.now() * 1e-6;

  private double mCurrentRotation = 0;
  private double mCurrentTranslationDirection = 0;
  private double mCurrentTranslationMagnitude = 0;

  private SlewRateLimiter mMagnitudeLimiter = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter mRotationLimiter = new SlewRateLimiter(Constants.DriveConstants.kRotationSlewRate);

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

  public void drive(double forwardSpeed, double leftSpeed, double rotation, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {

      double inputTranslationDirection = Math.atan2(leftSpeed, forwardSpeed);
      double inputTranslationMagnitude = Math.sqrt(forwardSpeed * forwardSpeed + leftSpeed * leftSpeed);

      double directionSlewRate;
      if (mCurrentTranslationMagnitude != 0) {
        directionSlewRate = Math.abs(Constants.DriveConstants.kDirectionSlewRate / mCurrentTranslationMagnitude);
      } else {
        directionSlewRate = 500.0;
      }

      double currentTime = WPIUtilJNI.now();
      double elapsedTime = currentTime - mPreviousTime;
      double angleDifference = SwerveUtils.angleDifference(inputTranslationDirection, mCurrentTranslationDirection);

      if (angleDifference < 0.45 * Math.PI) {
        mCurrentTranslationDirection = SwerveUtils.stepTowardsCircular(mCurrentTranslationDirection,
            inputTranslationDirection, directionSlewRate * elapsedTime);
        mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(inputTranslationMagnitude);
      } else if (angleDifference > 0.85 * Math.PI) {
        if (mCurrentTranslationMagnitude > 1e-4) {
          mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(0);
        } else {
          mCurrentTranslationDirection = SwerveUtils.wrapAngle(mCurrentTranslationDirection + Math.PI);
          mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(inputTranslationMagnitude);
        }
      } else {
        mCurrentTranslationDirection = SwerveUtils.stepTowardsCircular(mCurrentTranslationDirection,
            inputTranslationDirection, directionSlewRate * elapsedTime);
        mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(0);
      }

      mPreviousTime = currentTime;

      xSpeedCommanded = mCurrentTranslationMagnitude * Math.cos(mCurrentTranslationDirection);
      ySpeedCommanded = mCurrentTranslationMagnitude * Math.sin(mCurrentTranslationDirection);
      mCurrentRotation = mRotationLimiter.calculate(rotation);

    } else {
      xSpeedCommanded = forwardSpeed;
      ySpeedCommanded = leftSpeed;
      mCurrentRotation = rotation;

    }

    double xSpeedDelivered = xSpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double rotationDelivered = mCurrentRotation * Constants.DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));

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
