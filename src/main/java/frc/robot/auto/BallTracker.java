package frc.robot.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.logging.RobotLogger;
import frc.robot.RobotContainer;


public class BallTracker {
    NetworkTable m_ballDataTable;
    private final RobotLogger logger = RobotContainer.getLogger();

    public BallTracker() {
        m_ballDataTable = NetworkTableInstance.getDefault().getTable("ML/detections");
        if (m_ballDataTable == null) {
            logger.logError("Ball Data Table is null.");
            throw new RuntimeException("Null data entry");
        }
    }

    public String getBallCoordinates() {
        NetworkTableEntry networkTableEntry = m_ballDataTable.getEntry("label");
        BallCoordinates ballCoordinates = new BallCoordinates();
        ballCoordinates.setLabel(networkTableEntry.getString("test"));

        return ballCoordinates.getLabel();
    }
}
