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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.logging.RobotLogger;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BeamBreakSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShishkabotsEncoder;
import frc.robot.Constants;
import frc.robot.auto.BallTracker;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.BeamBreakSensor;


public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  private DriveTrain m_driveTrain;
  //private Arm m_arm;

  
  //Definitions for the hardware. Change this if you change what stuff you have plugged in

  VictorSPX intake = new VictorSPX(6);

  Joystick driverController = new Joystick(0);

  private final RobotLogger logger = RobotContainer.getLogger();
  private CameraSubsystem m_cam1;
  private ColorSensor m_colorSensor;

  private ShishkabotsEncoder m_encoder;

  private double autoStart = 0;
  private boolean goForAuto = false;

  private String m_driveMode;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private BallTracker m_ballTracker;

  private int m_logCounter;
  
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
      //m_arm = new Arm();
    
      //add a thing on the dashboard to turn off auto if needed
      SmartDashboard.putBoolean("Go For Auto", false);
      goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

    
    // Sets Limelight to driver camera, turn off green LEDs.
    m_cam1 = new CameraSubsystem();
    m_cam1.setCamToDriverMode();
    m_cam1.setLedToOff();

    m_colorSensor = new ColorSensor();

      m_chooser.setDefaultOption(Constants.ARCADE_DRIVE, Constants.ARCADE_DRIVE);
      m_chooser.addOption(Constants.TANK_DRIVE, Constants.TANK_DRIVE);
      m_chooser.addOption(Constants.CURVATURE_DRIVE, Constants.CURVATURE_DRIVE);
      SmartDashboard.putData("Drive Choices: ", m_chooser);

      m_ballTracker = new BallTracker();
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
  public void autonomousInit() {
    try {
      // get a time for auton start to do events based on time later
      autoStart = Timer.getFPGATimestamp();
      // check dashboard icon to ensure good to do auto
      goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    } catch (Exception e) {
      logger.logError("Runtime exception in autonomousInit " + e);
      throw e;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      if ((m_logCounter / 100.0) % 1 == 0) {
        if (m_ballTracker.chooseMostConfidentBall() != null) {
          logger.logInfo(m_ballTracker.chooseMostConfidentBall().toString());
        }
        else {
          logger.logInfo("No ball located!");
        }
      } // m_arm.commonPeriodic();
    
      // The code below is from everybot and will be moved out eventually, so no need to put into try-catch right now.
      // Get time since start of autonomous
      double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
      if(goForAuto){
        //series of timed events making up the flow of auto
        if(autoTimeElapsed < 3){
          //spit out the ball for three seconds
          intake.set(ControlMode.PercentOutput, -1);
        }else if(autoTimeElapsed < 6){
          //stop spitting out the ball and drive backwards *slowly* for three seconds
          intake.set(ControlMode.PercentOutput, 0);
          m_driveTrain.autonomousInit();
        }else{
          //do nothing for the rest of auto
          intake.set(ControlMode.PercentOutput, 0);
          m_driveTrain.autonomousEnd();
        }
      }
    } catch (Exception e) {
        logger.logError("Runtime Exception in autonomousPeriodic " + e);
        throw e;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      logger.logInfo("Teleop periodic started");
      if (m_ballTracker.chooseMostConfidentBall() != null) {
        SmartDashboard.putString("Most confident ball: ", m_ballTracker.chooseMostConfidentBall().toString());
      }
      else {
        SmartDashboard.putString("Most confident ball: ", "No ball located!");
      }

      m_driveMode = m_chooser.getSelected();
      m_robotContainer.setDriveType(m_driveMode);

      //Intake controls
      if(driverController.getRawButton(5)){
        intake.set(VictorSPXControlMode.PercentOutput, 1);;
      }
      else if(driverController.getRawButton(7)){
        intake.set(VictorSPXControlMode.PercentOutput, -1);
      }
      else{
        intake.set(VictorSPXControlMode.PercentOutput, 0);
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
    intake.set(ControlMode.PercentOutput, 0);
  }
}