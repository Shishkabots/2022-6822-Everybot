package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * WPILib's DigitalInput API:
 * @link{https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DigitalInput.html}
 */
public class BeamBreakSensor {
    private DigitalInput m_beamBreakSensor;

    public BeamBreakSensor() {
        m_beamBreakSensor = new DigitalInput(Constants.BEAM_BREAK_SENSOR_CHANNEL);
    }

    /**
     * Returns true if beam is uninterrupted, false if beam is obstructed.
     * @return
     */
    public boolean getValue() {
        return m_beamBreakSensor.get();
    }

    /**
     *  Puts value on SmartDashboard. (true/false)
     */
    public void putValueOnSmartDashboard() {
        SmartDashboard.putBoolean(Constants.IS_BEAM_INTERRUPTED, getValue());
    }
}
