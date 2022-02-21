package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class ShishkabotsEncoder {
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private double distancePerPulse;
    
    /**
     * Creates instances of left and right encoders. 
     * Sets the distance per pulse for the left and right encoders. 
     * @param distancePerPulse
     */
    public ShishkabotsEncoder(double distancePerPulse) {
        leftEncoder = new Encoder(1, 2, false, EncodingType.k4X);
        rightEncoder = new Encoder(3, 4, true, EncodingType.k4X); // Count direction reversed.
        this.distancePerPulse = distancePerPulse;
        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
    }

    /**
     * Resets the distance of all/both encoders to zero. 
     */
    public void Reset() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Finds the average distance of the two encoders.
     * @return average distance.
     */
    public double getDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
    }


}
