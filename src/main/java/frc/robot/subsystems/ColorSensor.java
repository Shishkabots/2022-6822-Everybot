package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.logging.RobotLogger;

public class ColorSensor extends SubsystemBase {
    private ColorSensorV3 m_colorSensor;
    private RobotLogger m_logger;

    public ColorSensor() {
        Port port = Port.kOnboard;
        m_colorSensor = new ColorSensorV3(port);
        
        m_logger = RobotContainer.getLogger();
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
        m_logger.logInfo("Red = " + m_colorSensor.getColor().red + "\nGreen = " + m_colorSensor.getColor().green + "\nBlue = " + m_colorSensor.getColor().blue + ".");
    }
} 