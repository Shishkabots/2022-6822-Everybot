package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class UltrasonicSensor {
    private AnalogInput m_ultrasonicSensor;

    /**
     * An ultrasonic sensor used to detect range.
     * @link{https://www.maxbotix.com/firstrobotics}
     * @param channel
     */
    public UltrasonicSensor(int channel) {
        m_ultrasonicSensor = new AnalogInput(Constants.ULTRASONIC_ANALOG_PORT);
    }

    public int getValue() {
        return m_ultrasonicSensor.getValue();
    }

    public double getRangeCM() {
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        return getValue() * voltage_scale_factor * Constants.ULTRASONIC_TO_CM_CONVERSION;
    }

    public double getRangeIN() {
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        return getValue() * voltage_scale_factor * Constants.ULTRASONIC_TO_IN_CONVERSION;
    }
}