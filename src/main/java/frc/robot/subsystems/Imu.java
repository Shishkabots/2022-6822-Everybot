package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.logging.RobotLogger;
import frc.robot.RobotContainer;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.ErrorCode;

public class Imu extends SubsystemBase {
    /**
     * Hardware manuel: https://store.ctr-electronics.com/content/user-manual/Pigeon%20IMU%20User's%20Guide.pdf
     * PigeonIMU API: https://docs.rs/ctre/0.6.1/ctre/sensors/pigeon/struct.PigeonIMU.html#method.get_yaw_pitch_roll
     */

    PigeonIMU m_pigeon = new PigeonIMU(0);
    private final RobotLogger logger = RobotContainer.getLogger();
    private double[] ypr;

    /*
    For reference:
    Yaw is the direction at which the robot is facing
    It is measured in degrees, not radians
    */


    public void initialize() {
        int counter = 0;
        //boot time is around 5 seconds, so may not be ready
        while(m_pigeon.getState() != PigeonIMU.PigeonState.Ready && counter < 5) {
            logger.logInfo("Pigeon IMU is not ready!");
            try {
                Thread.sleep(1000); 
            } catch(InterruptedException ex) {
                logger.logInfo("got interrupted!");
            }
            counter++;
        }
    }

    public double getYaw() {
        ypr = new double[3];
        m_pigeon.getYawPitchRoll(ypr); // https://docs.rs/ctre/0.6.1/ctre/sensors/pigeon/struct.PigeonIMU.html#method.get_yaw_pitch_roll
        SmartDashboard.putNumber("Robot Direction / yaw: ", ypr[0]);
        return ypr[0];
    }

    public double getPitch() {
        ypr = new double[3];
        m_pigeon.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Robot Pitch", ypr[1]);
        return ypr[1];
    }

    public double getRoll() {
        ypr = new double[3];
        m_pigeon.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Robot Roll: ", ypr[2]);
        return ypr[2];
    }
    public double getTemperature() {
        double temperature = m_pigeon.getTemp();
        // will require calibraton
        SmartDashboard.putNumber("Robot Temperature: ", temperature);
        return temperature;
    }

    public ErrorCode enterCalibrationMode(CalibrationMode calMode) {
        return(m_pigeon.enterCalibrationMode(calMode));
    }
}