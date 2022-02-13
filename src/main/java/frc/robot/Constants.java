// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // CAN IDs for all motors on robot
    public final static int DRIVETRAIN_RIGHT_FRONT_MOTOR = 1;
    public final static int DRIVETRAIN_RIGHT_BACK_MOTOR = 2;
    public final static int DRIVETRAIN_LEFT_FRONT_MOTOR = 3;
    public final static int DRIVETRAIN_LEFT_BACK_MOTOR = 4;
    // Here onwards are hypothetical values for probable CAN IDs for these motors.
    public final static int SHOOTER_LEAD_MOTOR = 5;
    public final static int SHOOTER_FOLLOWER_MOTOR = 6;
    public final static int INTAKE_LEAD_MOTOR = 7;
    public final static int INTAKE_FOLLOWER_MOTOR = 8;
    public final static int ELEVATOR_MOTOR = 9;

    public final static boolean IS_INVERTED = true;

    public final static int DRIVER_STICK_PORT = 0;

    // Joystick IDs    
    public final static int JOYSTICK_LEFT_X = 0;
    public final static int JOYSTICK_LEFT_Y = 1;
    public final static int JOYSTICK_RIGHT_X = 2;
    public final static int JOYSTICK_RIGHT_Y = 3;
    
    public final static int JOYSTICK_BUTTON_X = 1;  
    public final static int JOYSTICK_BUTTON_A = 2;      
    public final static int JOYSTICK_BUTTON_B = 3;  
    public final static int JOYSTICK_BUTTON_Y = 4; 
    public final static int JOYSTICK_LEFTBUMPER = 5;  
    public final static int JOYSTICK_RIGHTBUMPER = 6;      
    public final static int JOYSTICK_LEFTTRIGGER = 7;  
    public final static int JOYSTICK_RIGHTTRIGGER = 8; 
    public final static int JOYSTICK_BUTTON_LEFTSTICK = 11;  
    public final static int JOYSTICK_BUTTON_RIGHTSTICK = 12;      

    // Speed sensitivity
    public final static double JOYSTICK_FULLSPEED = 1.0;
    public final static double JOYSTICK_THROTTLESPEED = 0.5;

    // Retention time in hours for logs on the RoboRIO target
    public final static double LOG_EXPIRATION_IN_HRS = 48;

    // Distance calculations
    // Robot height and turret angle are ESTIMATES, NOT ACCURATE
    public final static double ROBOT_HEIGHT_IN_CM = 144.78;
    public final static double HUB_HEIGHT_IN_CM = 264;
    public final static double TURRET_ANGLE_IN_DEGREES = 30; 
    public final static double GRAVITY_M_PER_SEC_SQUARED = 9.81;

    /**
     * Utilities, such as encoder, and various conversions.
     * WHEEL_CIRCUMFERENCE_METERS calculation to be moved to another class.
     * ENCODER_UNITS_PER_REVOLUTION is just a placeholder.
     * GEARBOX_RATIO_OVER_ONE is a plcaeholder as well, to be finalized.
     * Sensor velocity is returned over 100ms, so SENSOR_TIME_IN_SECONDS shows it in seconds.
     */

    public final static double WHEEL_DIAMETER_INCHES = 6;
    public final static double ENCODER_UNITS_PER_REVOLUTION = 2048;
    public final static double GEARBOX_RATIO_OVER_ONE = 9;
    public final static double SENSOR_TIME_IN_SECONDS = 0.1;
    public final static double METER_TO_INCHES = 39.37;

    public final static String LIMELIGHT_X_OFFSET = "LimelightX Offset";
    public final static String LIMELIGHT_Y_OFFSET = "LimelightY Offset";
    public final static String LIMELIGHT_AREA_OFFSET = "LimelightArea Offset";

    public final static String SHOOTER_STATE_KEY = "Shooter state";

    public final static String SMARTDASHBOARD_KEY_TARGET_FLYWHEEL_SPEED = "Target flywheel speed";
    public final static double FLYWHEEL_DEFAULT_SPEED = 0.0;

    /**
     * ColorSensorV3 Constants
     */

    public final static double COLOR_SENSOR_PORT = 3;
    public final static Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    public final static Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    public final static Color kRedTarget = new Color(0.561, 0.232, 0.114);
    public final static Color kYellowTarget = new Color(0.361, 0.524, 0.113);
}