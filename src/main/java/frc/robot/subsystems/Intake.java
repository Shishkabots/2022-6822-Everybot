package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.subsystems.Motor;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class Intake extends SubsystemBase{
   /**
   * Creates a new Intake.
   */
    Motor intakeMotor;
    
    public Intake() {
       intakeMotor = new Motor(Constants.INTAKE_LEAD_MOTOR, MotorType.kBrushed);
    } 
    public void intakeBall() {
       intakeMotor.setSpeed(Constants.INTAKE_INTAKE_SPEED);
    }
    public void outtakeBall() {
        intakeMotor.setSpeed(Constants.INTAKE_OUTTAKE_SPEED);
    }
      
    public void stopIntake() {
        intakeMotor.setSpeed(0);
    }

    public void setSpeed(double speed) {
        intakeMotor.setSpeed(speed);
    }

}
