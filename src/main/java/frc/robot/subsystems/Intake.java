package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.subsystems.Motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class Intake extends SubsystemBase{
   /**
   * Creates a new Intake.
   */
    VictorSPX intakeMotor;
    
    public Intake(int canID) {
       intakeMotor = new VictorSPX(canID);
    } 
    public void intakeBall() {
       setSpeed(Constants.INTAKE_INTAKE_SPEED);
    }
    public void outtakeBall() {
        setSpeed(Constants.INTAKE_OUTTAKE_SPEED);
    }
      
    public void stopIntake() {
        setSpeed(0);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

}
