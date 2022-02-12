package frc.robot.subsystems;

import frc.robot.auto.LimelightCamera;
import frc.robot.auto.VisionConstants.camMode;

public class CameraSubsystem {
    public LimelightCamera m_limelightCamera;

    public CameraSubsystem() {
        m_limelightCamera = new LimelightCamera();
        m_limelightCamera.setCamMode(camMode.DRIVER_CAMERA_MODE);;
    }
}
