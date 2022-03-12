/*
  2022 everybot code
  written by carson graf 
  don't email me, @ me on discord
*/

/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.logging.RobotLogger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BeamBreakSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShishkabotsEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  
  private RobotContainer m_robotContainer;
  private DriveTrain m_driveTrain;
  private Arm m_arm;
  private VictorSPX m_intake;

  
  //Definitions for the hardware. Change this if you change what stuff you have plugged in

  Joystick m_driverStick = m_robotContainer.getDriverStick();

  private final RobotLogger logger = RobotContainer.getLogger();

  private ShishkabotsEncoder m_encoder;

  private double autoStart = 0;
  private boolean goForAuto = false;

  private String m_driveMode;
  private final SendableChooser<String> m_driveModeChooser = new SendableChooser<>();
  
  private BeamBreakSensor m_beamBreakSensor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try {
      // Log that robot has been initialized
      logger.logInfo("Robot initialized."); 
    
      m_robotContainer = new RobotContainer();
      m_driveTrain = m_robotContainer.getDriveTrain();
      m_arm = m_robotContainer.getArm();
      m_intake = new VictorSPX(6); //change? DG

      // Add a thing on the dashboard to turn off auto if needed
      SmartDashboard.putBoolean("Go For Auto", false);
      goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

      m_driveModeChooser.setDefaultOption(Constants.ARCADE_DRIVE, Constants.ARCADE_DRIVE);
      m_driveModeChooser.addOption(Constants.TANK_DRIVE, Constants.TANK_DRIVE);
      m_driveModeChooser.addOption(Constants.CURVATURE_DRIVE, Constants.CURVATURE_DRIVE);
      SmartDashboard.putData("Drive Choices: ", m_driveModeChooser);

      //Initializes the encoders. 
      m_encoder = new ShishkabotsEncoder(Constants.DISTANCE_PER_PULSE_Rev_11_1271);
    }  catch (Exception e) {
      logger.logError("Runtime Exception in robotInit" + e);
      throw e;
  }
}

    /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    try {
      CommandScheduler.getInstance().run();
    } catch (Exception e) {
        logger.logError("Runtime Exception in robotPeriodic " + e);
        throw e;
    }
  }

  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
       m_driveMode = m_driveModeChooser.getSelected();
       m_robotContainer.setDriveType(m_driveMode);

      //m_driveTrain.teleopPeriodic(-driverController.getRawAxis(1), -driverController.getRawAxis(2));
      //Intake controls
      if(m_driverStick.getRawButton(Constants.JOYSTICK_LEFTBUMPER)){
        m_intake.set(VictorSPXControlMode.PercentOutput, 1);;
      }
      else if(m_driverStick.getRawButton(Constants.JOYSTICK_LEFTTRIGGER)){
        m_intake.set(VictorSPXControlMode.PercentOutput, -1);
      }
      else{
        m_intake.set(VictorSPXControlMode.PercentOutput, 0);
      }

      // Will be uncommented when arm is ready.
      //m_arm.commonPeriodic();
  
      /**if(driverController.getRawButtonPressed(6) && !m_arm.getArmUpStatus()){
        m_arm.setLastBurstTime(Timer.getFPGATimestamp());
        m_arm.setArmUpStatus(true);
      }
      else if(driverController.getRawButtonPressed(8) && m_arm.getArmUpStatus()){
        m_arm.setLastBurstTime(Timer.getFPGATimestamp());
        m_arm.setArmUpStatus(false);
      }  */
    } catch (Exception e) {
      logger.logError("Runtime Exception in teleopPeriodic" + e);
      throw e;
    }
  }

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    //m_arm.disabledInit();
    m_intake.set(ControlMode.PercentOutput, 0);
  }
}