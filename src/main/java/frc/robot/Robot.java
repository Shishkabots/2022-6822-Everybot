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
<<<<<<< HEAD
import frc.robot.subsystems.Intake;
=======
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
>>>>>>> origin/main


public class Robot extends TimedRobot {
  public static Intake m_intake = new Intake();

  private RobotContainer m_robotContainer;
  private DriveTrain m_driveTrain;
  private Arm m_arm;

  
  //Definitions for the hardware. Change this if you change what stuff you have plugged in

  VictorSPX intake = new VictorSPX(6);

  Joystick driverController = new Joystick(0);

  private final RobotLogger logger = RobotContainer.getLogger();
  private CameraSubsystem cam1;
  private ColorSensor colorSensor;

  private double autoStart = 0;
  private boolean goForAuto = false;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Log that robot has been initialized
    logger.logInfo("Robot initialized."); 
    
    m_robotContainer = new RobotContainer();
    m_driveTrain = new DriveTrain();
    m_arm = new Arm();
    
    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

    
    // Sets Limelight to driver camera, turn off green LEDs.
    cam1 = new CameraSubsystem();
    cam1.setCamToDriverMode();
    cam1.setLedToOff();

    colorSensor = new ColorSensor();
  }

  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    m_arm.commonPeriodic();
    
    //get time since start of auto
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
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = -driverController.getRawAxis(2);
    
    m_driveTrain.teleopPeriodic(forward, turn);

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

    m_arm.commonPeriodic();
  
    if(driverController.getRawButtonPressed(6) && !m_arm.getArmUpStatus()){
      m_arm.setLastBurstTime(Timer.getFPGATimestamp());
      m_arm.setArmUpStatus(true);
    }
    else if(driverController.getRawButtonPressed(8) && m_arm.getArmUpStatus()){
      m_arm.setLastBurstTime(Timer.getFPGATimestamp());
      m_arm.setArmUpStatus(false);
    }  

  }

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    m_driveTrain.disabledInit();
    m_arm.disabledInit();
    intake.set(ControlMode.PercentOutput, 0);
  }
}