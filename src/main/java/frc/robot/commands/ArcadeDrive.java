// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.logging.RobotLogger;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_speed;
  private final DoubleSupplier m_rotation;
  private final RobotLogger logger = RobotContainer.getLogger();

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param  drivetrain The drivetrain used by this command.
   */
  public ArcadeDrive(DoubleSupplier speed, DoubleSupplier rotation, DriveTrain drivetrain, double sensitivity) {
    m_drivetrain = drivetrain;
    m_speed = () -> speed.getAsDouble() * sensitivity;
    m_rotation = () -> rotation.getAsDouble() * sensitivity;
    addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.logInfo("ArcadeDrive initialized");
   }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    m_drivetrain.arcadedrive(m_speed.getAsDouble(), m_rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.logInfo("Arcadedrive ended, interrupted = " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  
  }  
}