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

  /** Creates a new Arm. */
  public Arm() {
    m_armMotor = new Motor(5, MotorType.kBrushless);
    m_armMotor.setInverted(false);
    m_armMotor.setIdleMode(true);
    m_armMotor.burnFlash();
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
  public void bothPeriodic() {
    if(Constants.armUp){
        if(Timer.getFPGATimestamp() - Constants.lastBurstTime < Constants.armTimeUp){
          m_armMotor.setSpeed(Constants.armTravel);
        }
        else{
            m_armMotor.setSpeed(Constants.armHoldUp);
        }
      }
      else{
        if(Timer.getFPGATimestamp() - Constants.lastBurstTime < Constants.armTimeDown){
            m_armMotor.setSpeed(-1 * Constants.armTravel);
        }
        else{
            m_armMotor.setSpeed(-1 * Constants.armHoldUp);
        }
      }
  }

  /**
   * Arm motor stops when robot disabled.
   */
  public void disabledInit() {
    m_armMotor.setSpeed(0);
  }
}