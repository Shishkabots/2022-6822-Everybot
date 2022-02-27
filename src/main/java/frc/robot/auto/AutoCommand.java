// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.logging.RobotLogger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.BallTracker;

/** An example command that uses an example subsystem. */
public class AutoCommand extends CommandBase {

    private Imu m_pigeon;
    private DriveTrain m_driveTrain;
    private BallTracker m_ballTracker;

  private final RobotLogger logger = RobotContainer.getLogger();

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param  drivetrain The drivetrain used by this command.
   */
  public AutoCommand(Imu imu, DriveTrain driveTrain, BallTracker ballTracker) {
    m_pigeon = imu;
    m_driveTrain = driveTrain;
    m_ballTracker = ballTracker;
    addRequirements(m_pigeon, m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.logInfo("Autonomous Command initialized!");
    m_pigeon.enterCalibrationMode(CalibrationMode.Magnetometer360);
   }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
      chooseMostConfidentBall();
      turnToBall();
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


  public void chooseMostConfidentBall() {
    if (m_ballTracker.chooseMostConfidentBall() != null) {
        SmartDashboard.putString("Most confident ball: ", m_ballTracker.chooseMostConfidentBall().toString());
    }
    else {
        SmartDashboard.putString("Most confident ball: ", "No ball located!");
    }
}

  // +/- 10 degrees to allow for overshoot and undershot, will be redone with PID now.
  public void turnToBall() {
    if (Math.round(m_pigeon.getYaw()) >= SmartDashboard.getNumber(Constants.TARGET_DEGREES_KEY, 0.0) - 10 && Math.round(m_pigeon.getYaw()) <= SmartDashboard.getNumber(Constants.TARGET_DEGREES_KEY, 0.0) + 10) {
        logger.logInfo("Angle reached");
        SmartDashboard.putString("Direction: ", "None");
        m_driveTrain.setAutoTurnDirection(Constants.STOP_TURNING);
      }
      else if (m_pigeon.getYaw() < SmartDashboard.getNumber(Constants.TARGET_DEGREES_KEY, 0.0)) {
        SmartDashboard.putString("Direction: ", "Clockwise");
        m_driveTrain.setAutoTurnDirection(Constants.CLOCKWISE);
      }
      else if (m_pigeon.getYaw() > SmartDashboard.getNumber(Constants.TARGET_DEGREES_KEY, 0.0)) {
        SmartDashboard.putString("Direction: ", "Counter clockwise");
        m_driveTrain.setAutoTurnDirection(Constants.COUNTER_CLOCKWISE);
      }
      SmartDashboard.putNumber("Yaw: ", m_pigeon.getYaw());
  }
}