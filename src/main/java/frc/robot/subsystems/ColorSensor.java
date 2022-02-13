package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.logging.RobotLogger;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.Constants;

/**
 * ColorSensor Subsystem
 * Some code implemented from REV's Example Code
 * @link{https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java}
 */
public class ColorSensor extends SubsystemBase {
    private ColorSensorV3 m_colorSensor;
    private RobotLogger m_logger;
    private final ColorMatch m_colorMatcher;

    public ColorSensor() {
        Port port = Port.kOnboard;
        m_colorSensor = new ColorSensorV3(port);
        m_colorMatcher = new ColorMatch(); 

        m_logger = RobotContainer.getLogger();
        m_colorMatcher.addColorMatch(Constants.kBlueTarget);
        m_colorMatcher.addColorMatch(Constants.kGreenTarget);
        m_colorMatcher.addColorMatch(Constants.kRedTarget);
        m_colorMatcher.addColorMatch(Constants.kYellowTarget);

    }

    /**
     * Accessor methods
     */
    public ColorSensorV3 getColorSensor() {
        return m_colorSensor;
    }

    public Color getColor() {
        return m_colorSensor.getColor();
    }

    @Override
    public void periodic() {
        ColorMatchResult match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
 
        m_logger.logInfo(checkColor(match));
    }

    public String checkColor(ColorMatchResult match) {
        String colorString = "";
        if (match.color == Constants.kBlueTarget) {
            colorString = "Blue";
          } else if (match.color == Constants.kRedTarget) {
            colorString = "Red";
          } else if (match.color == Constants.kGreenTarget) {
            colorString = "Green";
          } else if (match.color == Constants.kYellowTarget) {
            colorString = "Yellow";
          } else {
            colorString = "Unknown";
          }
        return colorString;
    }
}  