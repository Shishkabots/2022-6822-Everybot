// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ColorSensor;

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
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.*;


/** An example command that uses an example subsystem. */
public class AutoCommand extends CommandBase {

    private Imu m_pigeon;
    private DriveTrain m_driveTrain;
    private BallTracker m_ballTracker;
    private BallCoordinates mostConfidentBallCoordinates;
    private ColorSensor m_colorSensor;
    private Intake m_intake;
    private Arm m_arm;

    private double kP = 0.3, kI = 0.3, kD = 1;
    private double integral, previous_error, error;
    private int setpoint = Constants.CAMERA_WIDTH_IN_PIXELS_OVER_TWO;
    private double rcw;
    private double imu_error;
    private double imu_rcw;
    // Arm is up when match starts
    private boolean armIsUp = true;

    private AutonomousState m_autonomousState;

    private enum AutonomousState {
      SCORE_PRELOADED_BALL, GO_TO_BALL, SCORE_BALL
    }

  private final RobotLogger logger = RobotContainer.getLogger();

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param  drivetrain The drivetrain used by this command.
   */
  public AutoCommand(Imu imu, DriveTrain driveTrain, BallTracker ballTracker, Arm arm, ColorSensor colorSensor) {
    m_pigeon = imu;
    m_driveTrain = driveTrain;
    m_ballTracker = ballTracker;
    m_arm = arm;
    m_colorSensor = colorSensor;
    addRequirements(m_pigeon, m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.logInfo("Autonomous Command initialized!");
    m_pigeon.enterCalibrationMode(CalibrationMode.Magnetometer360); //check if right calibration DG
   }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
      //checkAutonomousState(); will be uncommented when the full method is done. DG
      chooseMostConfidentBall();
      PIDBallTurningControl();
      turnToBall();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.logInfo("AutoCommand ended, interrupted = " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted

  } 

  public void checkAutonomousState() {
    switch (m_autonomousState) {
      case SCORE_PRELOADED_BALL:
        /*while (ballPickedUp()) {
          scoreBall();
        }*/
        SmartDashboard.putString("Autocommand state", "SCORE_PRELOADED_BALL");
        // Once the preloaded ball is dropped, set state to go to ball.
        m_autonomousState = AutonomousState.GO_TO_BALL;
        break;
      case GO_TO_BALL:
        SmartDashboard.putString("Autocommand state", "GO_TO_BALL");
        chooseMostConfidentBall();
        PIDBallTurningControl();
        turnToBall();
        goStraight();
        pickUpBall();
        if (ballPickedUp()) {
          m_autonomousState = AutonomousState.SCORE_BALL;
        }
        break;
      case SCORE_BALL:
        if (ballPickedUp()) {
          PIDHubTurningControl();
          turnToHub();
          // add rangesensor stop here 
          //while (m_rangeFinderSensor > 20 inches)
          //goStraight();
          //scoreBall();
        }
        break;
    }
  }

  public void chooseMostConfidentBall() {
    mostConfidentBallCoordinates = m_ballTracker.chooseMostConfidentBall();
    if (mostConfidentBallCoordinates != null) {
        SmartDashboard.putString("Most confident ball: ", mostConfidentBallCoordinates.toString());
    }
    else {
        SmartDashboard.putString("Most confident ball: ", "No ball located!");
    }
}

  /**
   * PID for turning
   * error is the target - actual.
   */
  public void PIDBallTurningControl(){
    try {
      error = setpoint - ((mostConfidentBallCoordinates.getXMin() + mostConfidentBallCoordinates.getXMax()) / 2); // Error = Target - Actual

      integral = (error * .02);
      //derivative = (error - previous_error) / .02;

      // If the current error is within the error leeway, the robot will stop turning.
      if (Math.abs(error) < Constants.ERROR_LEEWAY) {
        rcw = 0;
      }
      else {
        rcw = (kP * error + kI * integral);
      }

      //Sensitivity adjustment, since the rcw value originally is in hundreds (it is the pixel error + integral).
      // 10.0 is an arbitrary number for testing, no real meaning behind it.
      if (rcw < 0) {
        rcw = -1 * Math.sqrt(Math.abs(rcw)) / 10.0;
      }
      else {
        rcw = Math.sqrt(rcw) / 10.0;
      }
      SmartDashboard.putString("values", rcw + " is rcw. " + "error is " + kP + ". integral is " + kI);
    }
    catch (Exception e) {
      SmartDashboard.putString("error", "runtime in pidturn");
    }
  }

  // +/- 10 degrees to allow for overshoot and undershot, will be redone with PID now.
  // Always call after chooseMostConfidentBall(), else mostConfidentBallCoordinates will be null
  public void turnToBall() {
    try {
    if (mostConfidentBallCoordinates != null) {
      SmartDashboard.putNumber("rcw in autocommand", rcw);
      m_driveTrain.arcadedrive(0, rcw);
    }
      SmartDashboard.putNumber("Yaw: ", m_pigeon.getYaw());
    } catch (Exception e) {
        logger.logError("Runtime Exception while trying to use PID with ball turning " + e);
        throw e;
    }
  }

  public void PIDHubTurningControl() {
    try {
      // Invert the yaw so that the robot moves the right way (if the robot is pointed left the error is positive so the robot will turn right)
      imu_error = -1 * m_pigeon.getYaw();

      integral = (imu_error * .02);
      //derivative = (error - previous_error) / .02;

      // If the current error is within the error leeway, the robot will stop turning.
      if (Math.abs(imu_error) < Constants.ERROR_LEEWAY) {
        imu_rcw = 0;
      }
      else {
        imu_rcw = (kP * imu_error + kI * integral);
      }

      //Sensitivity adjustment, since the rcw value originally is in hundreds (it is the pixel error + integral).
      // 10.0 is an arbitrary number for testing, no real meaning behind it.
      if (imu_rcw < 0) {
        imu_rcw = -1 * Math.sqrt(Math.abs(imu_rcw)) / 10.0;
      }

      else {
        imu_rcw = Math.sqrt(imu_rcw) / 10.0;
      }
      SmartDashboard.putString("values", imu_rcw + " is rcw. " + "error is " + kP + ". integral is " + kI);
    }
    catch (Exception e) {
      SmartDashboard.putString("error", "runtime in pidturn");
    }
  }

  public void turnToHub() {
    try {
      SmartDashboard.putNumber("imu rcw", imu_rcw);
      m_driveTrain.arcadedrive(0, imu_rcw);
      if (m_pigeon.getYaw() > 0 - Constants.ERROR_LEEWAY || m_pigeon.getYaw() < 0 + Constants.ERROR_LEEWAY) {
      }
    } catch (Exception e) {
      logger.logError("Runtime Exception while trying to use PID with turning to hub " + e);
      throw e;
    }
  }

  /**
   * Goes straight forward.
   */
  public void goStraight() {
    try {
      m_driveTrain.arcadedrive(0.3, 0);
    } catch (Exception e) {
      logger.logError("Runtime Exception while trying to go straight " + e);
      throw e;
    }
  }

  public void pickUpBall() {
    double timeAtWhichProcessStarts = Timer.getFPGATimestamp();
    // If the timer since when the process starts is less than desired time (from everybot code) lower arm. ArmIsUp should be true too.
    while(Timer.getFPGATimestamp() < timeAtWhichProcessStarts + Constants.ARM_TIME_UP && armIsUp == true) {
      m_arm.setSpeed(-Constants.ARM_TRAVEL);
    }
    m_arm.setSpeed(-Constants.ARM_HOLD_DOWN);
    armIsUp = false;
  }

  public void putArmUp() {
    double timeAtWhichProcessStarts = Timer.getFPGATimestamp();
    // If the timer since when the process starts is less than desired time (from everybot code) lower arm. ArmIsUp should be true too.
    while(Timer.getFPGATimestamp() < timeAtWhichProcessStarts + Constants.ARM_TIME_DOWN && armIsUp == false) {
      m_arm.setSpeed(Constants.ARM_TRAVEL);
    }
    m_arm.setSpeed(Constants.ARM_HOLD_DOWN);
    armIsUp = true;
  }

  public void scoreBall() {
    putArmUp();
    if (armIsUp) {
      m_intake.setSpeed(Constants.SPEED_TO_SPIT_OUT_BALL);
    }
  }
  /**
   * Returns a value based on if the colorsensor detects a color or not. If it doesn't, it means there is no ball present.
   * TO BE TESTED.
   * @return
   */
  public boolean ballPickedUp() {
    return (m_colorSensor.checkColor().equals(""));
  }
}