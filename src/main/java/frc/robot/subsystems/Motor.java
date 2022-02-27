package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;

public class Motor {
    private CANSparkMax m_motor;

    public Motor(int deviceId, MotorType type, double kP, double kI, double kD) {
        m_motor = new CANSparkMax(deviceId, type);
        SparkMaxPIDController m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(kP);
        m_PIDController.setI(kI);
        m_PIDController.setD(kD);
        m_motor.burnFlash();
    } 

    public Motor(int deviceId, MotorType type) {
        m_motor = new CANSparkMax(deviceId, type);
    }

    /**
     * Inverts motor.
     * @param isInverted
     */
    public void setInverted(boolean isInverted) {
        m_motor.setInverted(isInverted);
    }

    /**
     * Returns underlying CANSparkMax motor
     * @return m_motor
     */
    public CANSparkMax getSparkMAXMotor() {
        return m_motor;
    }


    /**
     * Sets motor to IdleMode of either brake or coast. If you do not want to have an IdleMode, do not call this method
     * If isIdle == true, IdleMode will be set to brake.
     * If isIdle == false, IdleMode will be set to coast.
     */
    public void setIdleMode(boolean isIdleModeBrake) {
        if (isIdleModeBrake) {
            m_motor.setIdleMode(IdleMode.kBrake);
        }
        else {
            m_motor.setIdleMode(IdleMode.kCoast);
        }
    }
    
    /**
     * Sets speed of motor object, parameter can be -1.0 to 1.0.
     * @param speed
     */
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    /**
     * Burns flash for Motor object.
     */
    public void burnFlash() {
        m_motor.burnFlash();
    }

    /**
     * Clears sticky faults for Motor object.
     */
    public void clearFaults() {
        m_motor.clearFaults();
    }

    public void set(double speed) {
        m_motor.set(speed);
    }
}
