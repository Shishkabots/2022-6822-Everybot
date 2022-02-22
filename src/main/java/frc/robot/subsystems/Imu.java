package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Imu extends SubsystemBase {
    /**
     * Hardware manuel: https://store.ctr-electronics.com/content/user-manual/Pigeon%20IMU%20User's%20Guide.pdf
     * PigeonIMU API: https://robotpy.readthedocs.io/projects/ctre/en/stable/ctre/PigeonIMU.html
     */

    PigeonIMU m_pigeon = new PigeonIMU(0);

    /*
    For reference:
    Yaw is the direction at which the robot is facing
    It is measured in degrees, not radians
    */


    public void initialize() {
        int counter = 0;
        //boot time is around 5 seconds, so may not be ready
        while(m_pigeon.getState() != PigeonIMU.PigeonState.Ready && counter < 5) {
            System.out.println("PigeonIMU isn't ready");
            try {
                Thread.sleep(1000); 
            } catch(InterruptedException ex) {
                System.out.println("got interrupted!");
            }
            counter++;
        }
    }

    public double getDirection() {
        double[] ypr = new double[3];
        m_pigeon.getYawPitchRoll(ypr);
        // System.out.println("Pigeon Direction is: " + ypr[0]);
        SmartDashboard.putNumber("Robot Direction / yaw: ", ypr[0]);
        return ypr[0];
    }
    public double getTemperature() {
        double temperature = m_pigeon.getTemp();
        // will require calibraton
        SmartDashboard.putNumber("Robot Temperature: ", temperature);
        return temperature;
    }
}