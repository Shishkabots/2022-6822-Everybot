package frc.robot.subsystems;

import frc.robot.auto.LimelightCamera;
import frc.robot.auto.VisionConstants.camMode;
import frc.robot.auto.VisionConstants.ledMode;

public class CameraSubsystem {
    private LimelightCamera m_limelightCamera;

    public CameraSubsystem() {
        m_limelightCamera = new LimelightCamera();
    }

    public void setCamToDriverMode() {
        m_limelightCamera.setCamMode(camMode.DRIVER_CAMERA_MODE);
    }

    public void setLedToOff() {
        m_limelightCamera.setLedMode(ledMode.FORCE_OFF);
    }

    /**
     * Accessor methods
     */
    public LimelightCamera getLimelightCamera() {
        return m_limelightCamera;
    }
} 