package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.Constants;

public class ShishkabotsEncoder {
    private Encoder m_leftEncoder;
    private Encoder m_rightEncoder;
    private double distancePerPulse;
    
    /**
     * Creates instances of left and right encoders. 
     * Sets the distance per pulse for the left and right encoders. 
     * @param distancePerPulse
     */
    public ShishkabotsEncoder(double distancePerPulse) {
        m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_FIRST_CHANNEL, Constants.LEFT_ENCODER_SECOND_CHANNEL, false, EncodingType.k4X);
        m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_FIRST_CHANNEL, Constants.RIGHT_ENCODER_SECOND_CHANNEL, true, EncodingType.k4X); // Count direction reversed.
        this.distancePerPulse = distancePerPulse;
        m_leftEncoder.setDistancePerPulse(distancePerPulse);
        m_rightEncoder.setDistancePerPulse(distancePerPulse);
    }

    /**
     * Resets the distance of all/both encoders to zero. 
     */
    public void reset() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /**
     * Finds the average distance of the two encoders.
     * @return average distance.
     */
    public double getDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
    }
}
