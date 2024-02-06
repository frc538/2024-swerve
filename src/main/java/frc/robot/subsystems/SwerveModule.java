// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;
    private final SparkPIDController drivePID;
    private final SparkPIDController turnPID;

    private double angularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveId, int turnId, double offset){
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
        drivePID = driveMotor.getPIDController();
        turnPID = turnMotor.getPIDController();

        drivePID.setFeedbackDevice(driveEncoder);
        turnPID.setFeedbackDevice(turnEncoder);

        driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.DrivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.DriveVelocityConversionFactor);

        turnEncoder.setPositionConversionFactor(Constants.ModuleConstants.TurnPositionConversionFactor);
        turnEncoder.setVelocityConversionFactor(Constants.ModuleConstants.TurnVelocityConversionFactor);

        driveEncoder.setInverted(Constants.ModuleConstants.DriveEncoderInverted);
        turnEncoder.setInverted(Constants.ModuleConstants.TurnEncoderInverted);

        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(Constants.ModuleConstants.TurnPIDMinInput);
        turnPID.setPositionPIDWrappingMaxInput(Constants.ModuleConstants.TurnPIDMaxInput);

        drivePID.setP(Constants.ModuleConstants.DriveP);
        drivePID.setI(Constants.ModuleConstants.DriveI);
        drivePID.setD(Constants.ModuleConstants.DriveD);
        drivePID.setFF(Constants.ModuleConstants.DriveFF);
        drivePID.setOutputRange(Constants.ModuleConstants.DriveMinOutput, Constants.ModuleConstants.DriveMaxOutput);

        turnPID.setP(Constants.ModuleConstants.TurnP);
        turnPID.setI(Constants.ModuleConstants.TurnI);
        turnPID.setD(Constants.ModuleConstants.TurnD);
        turnPID.setFF(Constants.ModuleConstants.TurnFF);
        turnPID.setOutputRange(Constants.ModuleConstants.TurnMinOutput, Constants.ModuleConstants.TurnMaxOutput);
        
        driveMotor.setIdleMode(Constants.ModuleConstants.DriveIdle);
        turnMotor.setIdleMode(Constants.ModuleConstants.TurnIdle);

        driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.DriveCurrentLimit);
        turnMotor.setSmartCurrentLimit(Constants.ModuleConstants.TurnCurrentLimit);

        driveMotor.burnFlash();
        turnMotor.burnFlash();
    }
}
