// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final Motor m_armMotor;
  private boolean m_armUp; 
  private double m_lastBurstTime;


  /** Creates a new Arm. */
  public Arm() {
    m_armMotor = new Motor(5, MotorType.kBrushless);
    m_armMotor.setInverted(false);
    m_armMotor.setIdleMode(true);
    m_armMotor.burnFlash();

    m_armUp = true;
    m_lastBurstTime = 0;
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
   * Runs when Robot.java's autonomousPeriodic and teleopPeriodic runs.
   * Sets arm up or down based on timestamp.
   */
  public void commonPeriodic() {
    if(m_armUp){
        if(Timer.getFPGATimestamp() - m_lastBurstTime < Constants.ARM_TIME_UP){
          m_armMotor.setSpeed(Constants.ARM_TRAVEL);
        }
        else{
            m_armMotor.setSpeed(Constants.ARM_HOLD_UP);
        }
      }
      else{
        if(Timer.getFPGATimestamp() - m_lastBurstTime < Constants.ARM_TIME_DOWN){
            m_armMotor.setSpeed(-1 * Constants.ARM_TRAVEL);
        }
        else{
            m_armMotor.setSpeed(-1 * Constants.ARM_HOLD_UP);
        }
      }
  }

  /**
   * Arm motor stops when robot disabled.
   */
  public void disabledInit() {
    m_armMotor.setSpeed(0);
  }

  // Returns private boolean m_armUp
  public boolean getArmUpStatus() {
    return m_armUp;
  }

  // Sets private double m_lastBurstTime
  public void setLastBurstTime(double lastBurstTime) {
    m_lastBurstTime = lastBurstTime;
  }

  public void setSpeed(double speed) {
    m_armMotor.set(speed);
  }
}