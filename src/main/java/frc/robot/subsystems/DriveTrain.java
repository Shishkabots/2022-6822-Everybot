// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class DriveTrain extends SubsystemBase {

  //private final Motor m_leftFrontMotor;
  private final Motor m_leftBackMotor;

  private final Motor m_rightFrontMotor;
  //private final Motor m_rightBackMotor;

  private MotorControllerGroup m_leftSide;
  private final MotorControllerGroup m_rightSide;

  private final DifferentialDrive m_robotDrive;

  /**
   * @brief Arcade drive for differential drive platform.
   * @param xSpeed the robot's speed along x-axis
   * @param zRotation the robot's rotation rate around z-axis
   * test
   */
  public void arcadedrive(double xSpeed, double zRotation){
    SmartDashboard.putNumber("rcw = ", xSpeed);
    m_robotDrive.arcadeDrive(xSpeed, zRotation);
  }
  public void tankdrive(double leftSpeed, double rightSpeed) {
    m_robotDrive.tankDrive(leftSpeed, rightSpeed * -1);
  }
 
  public void curvaturedrive(double curveSpeed, double curveRotation, boolean isQuickTurn) {
    m_robotDrive.curvatureDrive(curveSpeed, curveRotation, isQuickTurn);
  }


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    
    //m_leftFrontMotor = new Motor(Constants.DRIVETRAIN_LEFT_FRONT_MOTOR, MotorType.kBrushed);
    m_leftBackMotor = new Motor(Constants.DRIVETRAIN_LEFT_BACK_MOTOR, MotorType.kBrushed);
    m_rightFrontMotor = new Motor(Constants.DRIVETRAIN_RIGHT_FRONT_MOTOR, MotorType.kBrushed);
    //m_rightBackMotor = new Motor(Constants.DRIVETRAIN_RIGHT_BACK_MOTOR, MotorType.kBrushed);

    //m_leftFrontMotor.clearFaults();
    m_leftBackMotor.clearFaults();
    m_rightFrontMotor.clearFaults();
    //m_rightBackMotor.clearFaults();

    /*m_leftFrontMotor.configFactoryDefault();
    m_leftBackMotor.configFactoryDefault();
    m_rightBackMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();*/

    //m_leftFrontMotor.setIdleMode(false);
    m_leftBackMotor.setIdleMode(false);
    m_rightFrontMotor.setIdleMode(false);
    //m_rightBackMotor.setIdleMode(false);

    /**
     * Inverts all motors based on value of the constant, which is currently true.
     * Currently the robot goes battery-first which is dangerous, so this will flip that direction.
     */
    m_rightFrontMotor.setInverted(Constants.IS_INVERTED);
    //m_rightBackMotor.setInverted(Constants.IS_INVERTED);

    
    m_leftSide = new MotorControllerGroup(m_leftBackMotor.getSparkMAXMotor());
    m_rightSide = new MotorControllerGroup(m_rightFrontMotor.getSparkMAXMotor());

    m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Program the motor speeds during teleop mode periodically. Modified from EveryBot code.
   * Turn and forward are the X and Y axis values respectively from the joystick.
   * @param forward
   * @param turn
   */
  public void teleopPeriodic(double forward, double turn) {

    double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;

    //m_leftFrontMotor.setSpeed(driveLeftPower);
    m_leftBackMotor.setSpeed(driveLeftPower);
    m_rightFrontMotor.setSpeed(driveRightPower);
    //m_rightBackMotor.setSpeed(driveRightPower);
  }

  public void disabledInit() {
    //m_leftFrontMotor.setSpeed(0);
    m_leftBackMotor.setSpeed(0);
    m_rightFrontMotor.setSpeed(0);
    //m_rightBackMotor.setSpeed(0);
  }



  public void setAutoTurnDirection(int direction) {
    /*if (direction == Constants.STOP_TURNING) {
        m_leftFrontMotor.set(0);
        m_rightSide.set(0);
    }*/
   if (direction == Constants.CLOCKWISE) {
      //m_leftFrontMotor.set(-0.15);
      m_rightFrontMotor.set(0.15);
      //m_rightBackMotor.set(0.15);
    }
    else if (direction == Constants.COUNTER_CLOCKWISE) {
        //m_leftFrontMotor.set(0.15);
        m_rightFrontMotor.set(-0.15);
        //m_rightBackMotor.set(-0.15);
    }
  }

  public void setEachMotorIndividually(double speed) {
    m_leftSide.set(speed);
    m_rightSide.set(speed);
  }
}