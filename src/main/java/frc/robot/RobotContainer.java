// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.BooleanSupplier;
import frc.robot.logging.RobotLogger;
import java.util.logging.Level;
import java.io.IOException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's commands, subsystems, and IO devices are defined here...
  public enum DriveType {
    ARCADE_DRIVE,
    TANK_DRIVE,
    CURVATURE_DRIVE
  }

  private Command m_autoCommand;
  private Command m_teleopCommand;
  private final DriveTrain m_drivetrain = new DriveTrain();
  private final Joystick m_driverStick = new Joystick(Constants.DRIVER_STICK_PORT);
  private DriveType m_driveType = DriveType.ARCADE_DRIVE;
  private static RobotLogger logger;
  
  // True makes it turn-in-place, false makes it do constant-curvature motion.
  private final BooleanSupplier m_isQuickTurn = () -> false; 
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // assign default commands
    m_drivetrain.setDefaultCommand(new ArcadeDrive(() -> (-m_driverStick.getRawAxis(Constants.JOYSTICK_LEFT_Y)), () -> m_driverStick.getRawAxis(Constants.JOYSTICK_RIGHT_X), m_drivetrain, Constants.JOYSTICK_THROTTLESPEED));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Removes speed throttling during ArcadeDrive, allows robot to move at max speed.
    new JoystickButton(m_driverStick, Constants.JOYSTICK_RIGHTTRIGGER).whenHeld(new ArcadeDrive(() -> (-m_driverStick.getRawAxis(Constants.JOYSTICK_LEFT_Y)), () -> m_driverStick.getRawAxis(Constants.JOYSTICK_RIGHT_X), m_drivetrain, Constants.JOYSTICK_FULLSPEED)); 
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getTeleopCommand(){
    return m_teleopCommand;
  }

  /**
   * Accessor methods 
   */
  public DriveType getDriveType() {
    return m_driveType;
  }

  // Initiates logger
    private static void initLogger(RobotLogger log) {
    try {
        log.init(RobotContainer.class);
        log.setLevel(Level.INFO);

        log.cleanLogs(Constants.LOG_EXPIRATION_IN_HRS);
        log.logInfo("Logger initialized");
    } 
    catch (IOException error) {
      log.logError("Failed to init logger!");
      throw new RuntimeException(error);
    }
  }

  //Gets logger
  public static RobotLogger getLogger() {
    if (logger == null) {
      logger = new RobotLogger();
      initLogger(logger);
    }
    return logger;
  }

  public DriveTrain getDriveTrain() {
    return m_drivetrain;
  }

   public void checkDrivetype() {
    switch(m_driveType) {
      case ARCADE_DRIVE:
        m_drivetrain.setDefaultCommand(new ArcadeDrive(() -> (-m_driverStick.getRawAxis(Constants.JOYSTICK_RIGHT_X)), () -> -m_driverStick.getRawAxis(Constants.JOYSTICK_LEFT_Y), m_drivetrain, Constants.JOYSTICK_THROTTLESPEED));
        SmartDashboard.putNumber("test arcade", m_driverStick.getRawAxis(Constants.JOYSTICK_LEFT_Y)); 
        break;
      case TANK_DRIVE:
        m_drivetrain.setDefaultCommand(new TankDrive(() -> (m_driverStick.getRawAxis(Constants.JOYSTICK_LEFT_Y)), () -> m_driverStick.getRawAxis(Constants.JOYSTICK_RIGHT_Y), m_drivetrain));
        break;
      case CURVATURE_DRIVE:
        m_drivetrain.setDefaultCommand(new CurvatureDrive(() -> (-m_driverStick.getRawAxis(Constants.JOYSTICK_RIGHT_Y)), () -> m_driverStick.getRawAxis(Constants.JOYSTICK_LEFT_Y), m_isQuickTurn, m_drivetrain));
        break;
      default:
        m_drivetrain.setDefaultCommand(new ArcadeDrive(() -> (-m_driverStick.getRawAxis(Constants.JOYSTICK_RIGHT_X)), () -> m_driverStick.getRawAxis(Constants.JOYSTICK_LEFT_Y), m_drivetrain, Constants.JOYSTICK_THROTTLESPEED)); 
    }
  }

  /**
   * Sets the drive type based on the string passed in (called in after getting SendableChooser in Robot.java)
   * Also runs the checkDriveType() method, to change the drivetype if necessary.
   */ 
  public void setDriveType(String driveType) {
    if(Constants.TANK_DRIVE.equalsIgnoreCase(driveType)) {  
      m_driveType = DriveType.TANK_DRIVE;
    }
    else if(Constants.CURVATURE_DRIVE.equalsIgnoreCase(driveType)) {
      m_driveType = DriveType.CURVATURE_DRIVE;
    }
    else {
      m_driveType = DriveType.ARCADE_DRIVE;
    }

    checkDrivetype();
  }
 } 