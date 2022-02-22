package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends SubsystemBase{
   /**
   * Creates a new Intake.
   */
    CANSparkMax intakeSpark;

    double intakeSpeed, outtakeSpeed;
    
    public Intake() {
       intakeSpark = new CANSparkMax(Constants.INTAKE_LEAD_MOTOR, MotorType.kBrushed);

       intakeSpeed = Constants.INTAKE_INTAKE_SPEED;
       outtakeSpeed = Constants.INTAKE_OUTTAKE_SPEED;
    } 
    public void intakeBall() {
       intakeSpark.set(intakeSpeed);
    }
    public void outtakeBall() {
        intakeSpark.set(outtakeSpeed);
    }
      
    public void stopIntake() {
        intakeSpark.set(0);
    }

}
