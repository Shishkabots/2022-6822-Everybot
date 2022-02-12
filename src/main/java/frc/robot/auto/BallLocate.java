package frc.robot.auto;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;
import frc.robot.subsystems.CameraSubsystem;


public class BallLocate extends CommandBase {

    private LimelightCamera m_limelight;
    private double distanceToBall;

    public BallLocate(CameraSubsystem cam_1) {
        m_limelight = cam_1.getLimelightCamera();
    }

    // Update smart dashboard with current offset values
    private void updateSmartDash() {
        SmartDashboard.putNumber(Constants.LIMELIGHT_X_OFFSET, m_limelight.getTx());
        SmartDashboard.putNumber(Constants.LIMELIGHT_Y_OFFSET, m_limelight.getTy());
        SmartDashboard.putNumber(Constants.LIMELIGHT_AREA_OFFSET, m_limelight.getTa());
    }

    /**
     * Calculate distance based on hub height, robot height, turret angle and angle offset from center. 
     * @link {https://docs.limelightvision.io/en/latest/cs_estimating_distance.html}
     * From diagram above:
     * @param angleAboveCameraInDegrees = a2
     * Constants.TURRET_ANGLE_IN_DEGREES = a1
     * Constants.HUB_HEIGHT_IN_CM = h2
     * Constants.ROBOT_HEIGHT_IN_CM = h1
     */
    public double calculateDistance(double angleAboveCameraInDegrees) {
        // Change constant names, need to reconfigure algorithm.
        distanceToBall = (Constants.HUB_HEIGHT_IN_CM - Constants.ROBOT_HEIGHT_IN_CM)/(Math.tan(Constants.TURRET_ANGLE_IN_DEGREES + angleAboveCameraInDegrees));
        return distanceToBall;
    }

    // Periodically run approximately every 20ms.
    public void execute() {
        // Updating offset metrics and area
        updateSmartDash();



        /**
         * Hard coded for now, will test on limelight and update. Call to limelight angle goes here.
         * Limelight requires 5 or 10 amps fuse, which has been ordered and fit Jan 23. Necessary wire still arriving.
         */
        double angleAboveCameraInDegrees = 10;

        // Calculates distance periodically and target flywheel speed.
        SmartDashboard.putNumber("Distance to hub", calculateDistance(angleAboveCameraInDegrees));
    }
}