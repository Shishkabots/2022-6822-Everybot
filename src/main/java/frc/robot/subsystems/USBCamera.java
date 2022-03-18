package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;


public class USBCamera {
    private UsbCamera m_camera;

    public USBCamera() {

        m_camera = new UsbCamera("Driver Camera", 1);
    }

    /**
     * Accessor methods
     */
    public UsbCamera getCamera() {
        return m_camera;
    }

    public String getPath() {
        return m_camera.getPath();
    }
} 