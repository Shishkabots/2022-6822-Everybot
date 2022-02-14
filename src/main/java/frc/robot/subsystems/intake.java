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

    double intakeSpeed;
    double outtakeSpeed; //is this needed?
    
    public Intake() {
       intakeSpark = new CANSparkMax(Constants.INTAKE_LEAD_MOTOR, MotorType.kBrushed);

       intakeSpeed = 0.5;
       outtakeSpeed = -0.5;
    } 
    public void IntakeBall() {
       intakeSpark.set(intakeSpeed);
    }
    public void OuttakeBall() {
        intakeSpark.set(outtakeSpeed);
    }
      
    public void StopIntake() {
        intakeSpark.set(0);
    }

}
