// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax m_leftFrontMotor;
  private final CANSparkMax m_leftBackMotor;

  private final CANSparkMax m_rightFrontMotor;
  private final CANSparkMax m_rightBackMotor;

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
    
    m_leftFrontMotor = new CANSparkMax(Constants.DRIVETRAIN_LEFT_FRONT_MOTOR, MotorType.kBrushed);
    m_leftBackMotor = new CANSparkMax(Constants.DRIVETRAIN_LEFT_BACK_MOTOR, MotorType.kBrushed);
    m_rightFrontMotor = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_FRONT_MOTOR, MotorType.kBrushed);
    m_rightBackMotor = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_BACK_MOTOR, MotorType.kBrushed);

    /**
     * Inverts all motors based on value of the constant, which is currently true.
     * Currently the robot goes battery-first which is dangerous, so this will flip that direction.
     */
    m_leftFrontMotor.setInverted(Constants.IS_INVERTED);
    m_leftBackMotor.setInverted(Constants.IS_INVERTED);
    m_rightFrontMotor.setInverted(Constants.IS_INVERTED);
    m_rightBackMotor.setInverted(Constants.IS_INVERTED);

    
    m_leftSide = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    m_rightSide = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

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

}