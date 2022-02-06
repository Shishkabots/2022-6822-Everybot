// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.logging.RobotLogger;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class CurvatureDrive extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_speed;
  private final DoubleSupplier m_rotation;
  private final BooleanSupplier m_isQuickTurn;
  private final RobotLogger logger = RobotContainer.getLogger();

      /**
        * Creates a new CurvatureDrive command.
        *
        * @param drivetrain The drivetrain used by this command.
        */
    public CurvatureDrive(DoubleSupplier speed, DoubleSupplier rotation, BooleanSupplier isQuickTurn, DriveTrain drivetrain) {
        m_drivetrain = drivetrain;
        m_speed = speed;
        m_rotation = rotation;
        m_isQuickTurn = isQuickTurn;
        addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.logInfo("CurvatureDrive initialized");
   }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    m_drivetrain.curvaturedrive(m_speed.getAsDouble(), m_rotation.getAsDouble(), m_isQuickTurn.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.logInfo("CurvatureDrive interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  } 
}