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
import frc.robot.subsystems.UltrasonicSensor;

/** An example command that uses an example subsystem. */
public class AutoCommand extends CommandBase {

    private Imu m_pigeon;
    private DriveTrain m_driveTrain;
    private BallTracker m_ballTracker;
    private BallCoordinates mostConfidentBallCoordinates;
    private ColorSensor m_colorSensor;
    private Intake m_intake;
    private Arm m_arm;
    private UltrasonicSensor m_ultrasonicSensor;

    private double kP = 0.3, kI = 0.3, kD = 1;
    private double derivative, previous_error, error;
    private int setpoint = Constants.CAMERA_WIDTH_IN_PIXELS_OVER_TWO;
    private double rcw;
    private double imu_error, imu_rcw, imu_derivative, imu_previous_error;
    // Arm is up when match starts
    private boolean armIsUp;

    private AutonomousState m_autonomousState;

    private enum AutonomousState {
      SCORE_PRELOADED_BALL, GO_TO_BALL, SCORE_BALL, DUMB_AUTO
    }

  private final RobotLogger logger = RobotContainer.getLogger();

  /**
   * Creates a new ArcadeDrive command.
   *
   * @param  drivetrain The drivetrain used by this command.
   */
  public AutoCommand(Imu imu, DriveTrain driveTrain, BallTracker ballTracker, Arm arm, UltrasonicSensor ultrasonicSensor, ColorSensor colorSensor) {
    m_pigeon = imu;
    m_driveTrain = driveTrain;
    m_ballTracker = ballTracker;
    m_arm = arm;
    m_ultrasonicSensor = ultrasonicSensor;
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
      checkAutonomousState();
      /*chooseMostConfidentBall();
      PIDBallTurningControl();
      turnToBall();*/ //will be removed when checkAutonomousState is confirmed to work DG
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
        SmartDashboard.putString(Constants.AUTOCOMMAND_KEY, "SCORE_PRELOADED_BALL");
        armIsUp = true;
        while (isBallPickedUp()) {
          scoreBall();
        }
        // Once the preloaded ball is dropped, set state to go to ball.
        m_autonomousState = AutonomousState.GO_TO_BALL;
        break;
      case GO_TO_BALL:
        SmartDashboard.putString(Constants.AUTOCOMMAND_KEY, "GO_TO_BALL");
        chooseMostConfidentBall();
        // DG - maybe remove this below line? Might induce unnecessary stopping of robot when the PID would just have it turn back to find the ball or when ball falls out of frmae when too near
        if (mostConfidentBallCoordinates != null) {
          PIDBallTurningControl();
          turnToBall();
          goStraight();
          pickUpBall();
        }
        if (isBallPickedUp()) {
          double timeWhenEnteredThisLoop = Timer.getFPGATimestamp();
          while (Timer.getFPGATimestamp() < timeWhenEnteredThisLoop + 0.5) {
            goStraight();
          }
          m_autonomousState = AutonomousState.SCORE_BALL;
        }
        break;
      case SCORE_BALL:
        SmartDashboard.putString(Constants.AUTOCOMMAND_KEY, "SCORE_BALL");
        if (isBallPickedUp()) {
          PIDHubTurningControl();
          turnToHub();
          if (m_ultrasonicSensor.getRangeIN() > Constants.BALL_DROP_DISTANCE_INCHES) {
            goStraight();
          }
          else {
            stopMoving();
            scoreBall();
            m_autonomousState = AutonomousState.GO_TO_BALL;
          }
        }
        else {
          m_autonomousState = AutonomousState.GO_TO_BALL;
        }
        break;
      case DUMB_AUTO:
        // This only works if the robot is placed directly aligned with the ball. It will pick up other ball and then turn around and score both.
        SmartDashboard.putString(Constants.AUTOCOMMAND_KEY, "DUMB_AUTO");
        // Goes straight until the ball is picked up.
        if (isBallPickedUp() == false) {
          pickUpBall();
          goStraight();
          if (isBallPickedUp()) {
            stopMoving();
          }
        }

        if (isBallPickedUp()) {
          //add stop moving in here to make it stop going forward? Don't know - DG
          PIDHubTurningControl();
          turnToHub();
          if (m_ultrasonicSensor.getRangeIN() > Constants.BALL_DROP_DISTANCE_INCHES) {
            goStraight();
          }
          else {
            scoreBall();
          }
          
        }
    }
  }

  public void chooseMostConfidentBall() {
    // DG - potential error here? -> if the ball goes out of frame after being scanned, robot may continue turning under guise that ball is present
    // However, adding an if statement to set mostConfidentBallCoordinates to null if the ballTracker.chooseMostConfidentBall() returns null may make the program default to spin in a direction
    // The current statement would have it spin back to where it last saw the ball relative to center and this should have it hopefully correct itself and find ball again.
    mostConfidentBallCoordinates = m_ballTracker.chooseMostConfidentBall();
    if (mostConfidentBallCoordinates != null) {
      SmartDashboard.putString("Most confident ball: ", mostConfidentBallCoordinates.toString());
    }
    else {
      // DG Turns clockwise slowly if no ball found
      m_driveTrain.arcadedrive(0, Constants.TURN_SPEED);
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

      // integral = (error * .02); DG add back if derivative doesn't work

      derivative = (error - previous_error) / 0.02;

      // If the current error is within the error leeway, the robot will stop turning.
      if (Math.abs(error) < Constants.ERROR_LEEWAY) {
        rcw = 0;
      }
      else {
        rcw = (kP * error + kD * derivative);
      }

      //Sensitivity adjustment, since the rcw value originally is in hundreds (it is the pixel error + integral).
      // 10.0 is an arbitrary number for testing, no real meaning behind it.
      if (rcw < 0) {
        rcw = -1 * Math.sqrt(Math.abs(rcw)) / 10.0;
      }
      else {
        rcw = Math.sqrt(rcw) / 10.0;
      }
      previous_error = error;
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

      //integral = (imu_error * .02); DG add back if derivative doesn't work
      imu_derivative = (imu_error - imu_previous_error) / 0.02;

      // If the current error is within the error leeway, the robot will stop turning.
      if (Math.abs(imu_error) < Constants.ERROR_LEEWAY) {
        imu_rcw = 0;
      }
      else {
        imu_rcw = (kP * imu_error + kD * imu_derivative);
      }

      //Sensitivity adjustment, since the rcw value originally is in hundreds (it is the pixel error + integral).
      // 10.0 is an arbitrary number for testing, no real meaning behind it.
      if (imu_rcw < 0) {
        imu_rcw = -1 * Math.sqrt(Math.abs(imu_rcw)) / 10.0;
      }

      else {
        imu_rcw = Math.sqrt(imu_rcw) / 10.0;
      }
      imu_previous_error = imu_error;
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
      m_driveTrain.arcadedrive(Constants.GO_STRAIGHT_SPEED, 0);
    } catch (Exception e) {
      logger.logError("Runtime Exception while trying to go straight " + e);
      throw e;
    }
  }

  // Stops motion of robot
  public void stopMoving() {
    try {
      m_driveTrain.arcadedrive(0, 0);
    } catch (Exception e) {
      logger.logError("Runtime Exception while trying to stop motion " + e);
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
  public boolean isBallPickedUp() {
    return (m_colorSensor.checkColor().equals(""));
  }
}