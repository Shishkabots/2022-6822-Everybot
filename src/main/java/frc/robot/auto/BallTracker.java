package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import org.json.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.logging.RobotLogger;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class BallTracker {
    NetworkTable m_ballDataTable;
    private final RobotLogger logger = RobotContainer.getLogger();

    public BallTracker() {
        m_ballDataTable = NetworkTableInstance.getDefault().getTable("ML");
        if (m_ballDataTable == null) {
            logger.logError("Ball Data Table is null.");
            throw new RuntimeException("Null data entry");
        }
    }

    /**
     * Returns list of ALL detected balls, and their respective labels, coordinates, and confidence levels
     * @return
     */
    private List<BallCoordinates> getBallCoordinates() {

        // Gets detections from the NetworkTable "ML/detections" where JSON strings are created.
        String networkTableEntryAsString = m_ballDataTable.getEntry("detections").getString(null);

        if (networkTableEntryAsString == null) {
            SmartDashboard.putString("Ball Data status: ", "Image Coordinate Table entry is null");
            return null;
        }
        JSONArray jsonArray = new JSONArray(networkTableEntryAsString);

        List<BallCoordinates> ballCoordinatesList = new ArrayList<>();

        for (int i = 0; i < jsonArray.length(); i++) {
            JSONObject jsonObject = jsonArray.getJSONObject(i);
            JSONObject boxJsonObject = jsonObject.getJSONObject("box");
            BallCoordinates ballCoordinates = new BallCoordinates();

            try {
                String label = jsonObject.getString("label");
                int ymin = boxJsonObject.getInt("ymin");
                int xmin = boxJsonObject.getInt("xmin");
                int ymax = boxJsonObject.getInt("ymax");
                int xmax = boxJsonObject.getInt("xmax");
                double confidence = jsonObject.getDouble("confidence");

                ballCoordinates.setLabel(label);
                ballCoordinates.setYMin(ymin);
                ballCoordinates.setXMin(xmin);
                ballCoordinates.setYMax(ymax);
                ballCoordinates.setXMax(xmax);
                ballCoordinates.setConfidence(confidence);
                ballCoordinatesList.add(ballCoordinates);
            }
            catch (JSONException e) {
                logger.logError("Exception processing JSON." + e);
                return null;
            }
        }
        return ballCoordinatesList;
    }

    /**
     * Cycles through the ballCoordinatesList returned from getBallCoordinates() to find the most confident ball and return that BallCoordinate.
     * @return
     */
    public BallCoordinates chooseMostConfidentBall() {
        List<BallCoordinates> ballCoordinatesList = getBallCoordinates();

        if (ballCoordinatesList == null || ballCoordinatesList.size() == 0) {
            return null;
        }

        // Initialize finalBallCoords to a non-null value
        BallCoordinates finalBallCoords = ballCoordinatesList.get(0);

        // Finds the best ball based on confidence level
        for (int i = 1; i < ballCoordinatesList.size(); i++) {
            if (ballCoordinatesList.get(i).getConfidence() > finalBallCoords.getConfidence()){
                finalBallCoords = ballCoordinatesList.get(i);
            }
        }
        return finalBallCoords;
    }
} 