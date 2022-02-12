package frc.robot.auto;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

/**
 * Limelight Camera class to provide accessor methods throughout project
 * Provides accessor methods for all Limelight NetworkTableEntry functions.
 * 
 * https://docs.limelightvision.io/en/latest/networktables_api.html
 */
public class LimelightCamera {
    NetworkTable m_table;

    public LimelightCamera() {
        m_table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // Return whether limelight has any valid targets (0 or 1)
    public double getTz() {
        return m_table.getEntry("tz").getDouble(0.0d);
    }
    
    // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    public double getTx() {
        return m_table.getEntry("tx").getDouble(0.0d);
    }

    // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    public double getTy() {
        return m_table.getEntry("ty").getDouble(0.0d);
    }

    // Target Area (0% of image to 100% of image)
    public double getTa() {
        return m_table.getEntry("ta").getDouble(0.0d);
    }

    //Skew or rotation (-90 degrees to 0 degrees)
    public double getTs() {
        return m_table.getEntry("ts").getDouble(0.0d);
    }

    //The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    public double getTl() {
        return m_table.getEntry("tl").getDouble(0.0d);
    }

    // Sidelength of shortest side of the fitted bounding box (pixels)
    public double getTshort() {
        return m_table.getEntry("tshort").getDouble(0.0d);
    }

    // Sidelength of longest side of the fitted bounding box (pixels)
    public double getTlong() {
        return m_table.getEntry("tlong").getDouble(0.0d);
    }

    // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
    public double getThor() {
        return m_table.getEntry("thor").getDouble(0.0d);
    }

    // Vertical sidelength of the rough bounding box (0 - 320 pixels)
    public double getTvert() {
        return m_table.getEntry("tvert").getDouble(0.0d);
    }

    // True active pipeline index of the camera (0 .. 9)
    public double getPipe() {
        return m_table.getEntry("getpipe").getDouble(0.0d);
    }

    // Set camera mode to driver cam.
    public void setCamMode(VisionConstants.camMode cMode) {
        m_table.getEntry("camMode").setNumber(cMode.value);
    }

    // Set LED mode.
    public void setLedMode(VisionConstants.ledMode lMode) {
        m_table.getEntry("ledMode").setNumber(lMode.value);
    }
}  