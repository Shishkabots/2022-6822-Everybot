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

    public pigeonIMU(double startingX, double startingY, double startingDirection) {
        initialX = startingX;
        initialY = startingY;
        initialDirection = startingDirection;
    }
    public void initialize() {
        int counter = 0;
        //boot time is around 5 seconds, so may not be ready
        while(pigeon.getState() != PigeonIMU.PigeonState.Ready && counter < 1000) {
            System.out.println("PigeonIMU isn't ready");
            counter++;
        }
    }

    public double getDirection() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        // System.out.println("Pigeon Direction is: " + ypr[0]);
        return ypr[0];
    }
    public double getTemperature() {
        // will require calibraton
        return pigeon.getTemp();
    }

    

}
