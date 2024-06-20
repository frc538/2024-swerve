// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final SwerveDriveSubsystem mDriveSubsystem = new SwerveDriveSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.button(1).whileTrue(mDriveSubsystem.setXCommand());
    m_driverController.button(2).onTrue(mDriveSubsystem.toggleFieldRelative());
    m_driverController.button(7).or(m_driverController.button(8)).onTrue(mDriveSubsystem.resetGyroCommand());

    mDriveSubsystem.setDefaultCommand(Commands.run(() -> {
      double forwardSpeed = Math.pow(-m_driverController.getY(), 3);
      double leftSpeed = Math.pow(-m_driverController.getX(), 3);
      double counterclockwiseSpeed = Math.pow ( -m_driverController.getZ(), 3);

      if (Math.abs(forwardSpeed) < 0.1) forwardSpeed = 0;
      if (Math.abs(leftSpeed) < 0.1) leftSpeed = 0;
      if (Math.abs(counterclockwiseSpeed) < 0.1) counterclockwiseSpeed = 0;

      mDriveSubsystem.drive(forwardSpeed, leftSpeed, counterclockwiseSpeed, true);
    }, mDriveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
