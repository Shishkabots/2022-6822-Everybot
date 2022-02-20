package frc.robot.subsystems;
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.PigeonIMU;

public class pigeonIMU extends SubsystemBase {
    double initialX;
    double initialY;
    double initialDirection;


    PigeonIMU pigeon = new PigeonIMU(0);

    /*
    For reference:
    Yaw is robot direction
    */

    public void PIgeonIMU(double startingX, double startingY, double startingDirection) {
        initialX = startingX;
        initialY = startingY;
        initialDirection = startingDirection;
    }
    public enum PigeonState {
        NoComm,
        Initializing,
        Ready,
        UserCalibration,
    }
    public void initialize() {
        int counter = 0;
        while(pigeon.getState() != PigeonState.Ready && counter < 100) {
            System.out.println("PigeonIMU isn't ready");
            counter++;
        }
    }

    public double getDirection() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        System.out.println("Pigeon Direction is: " + ypr[0]);
    }
    public double getX() {

    }

    

}
