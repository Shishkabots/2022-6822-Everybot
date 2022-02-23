package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

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


}
