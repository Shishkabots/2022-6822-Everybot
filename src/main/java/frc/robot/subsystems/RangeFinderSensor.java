import edu.wpi.first.wpilibj.Ultrasonic;

public class RangeFinderSensor {
    private Ultrasonic m_rangeFinderSensor;

    /**
     * A rangefinder using ultrasonic to detect range.
     * @link{https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Ultrasonic.html}
     * @param channel
     */
    public RangeFinderSensor(int channel) {
        m_rangeFinderSensor = new Ultrasonic(Constants.PING_CHANNEL, Constants.ECHO_CHANNEL);

    }

    public double getRangeInches() {
        return m_rangeFinderSensor.getRangeInches();
    }

    public double getRangeMM() {
        return m_rangeFinderSensor.getRangeMM()
    }

    public void setEnabled(boolean enable) {
        m_rangeFinderSensor.setEnabled(enable);
    }

    public boolean isRangeValid() {
        m_rangeFinderSensor.isRangeValid();
    }

    public void closeSensor() {
        m_rangeFinderSensor.close();
    }

    public void setAutomaticModeForAllSensors(boolean enabling) {
        Ultrasonic.setAutomaticMode(enabling);
    }
}
