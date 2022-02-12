package frc.robot.auto;

/**
 * The VisionConstants class provides a convenient place for teams to hold vision-wide numerical or boolean
 * vision constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public class VisionConstants {

    // ledMode - Sets limelight’s LED state
    public static enum ledMode {
       USE_LED_MODE(0), 
       FORCE_OFF(1),
       FORCE_BLINK(2),
       FORCE_ON(3);

       public final int value;
       public final static String name = "ledMode";
       private ledMode(int val){
           this.value = val;
       }     
    }

    // camMode - Sets limelight’s operation mode
    public static enum camMode {
        VISION_PROCESSOR_MODE(0),
        DRIVER_CAMERA_MODE(1);

        public final int value;
        private camMode(int val){
            this.value = val;
        }     
    }

    // pipeline - Sets limelight’s current pipeline
    public static enum pipeline {
        ZERO(0), ONE(1), TWO(2), THREE(3), FOUR(4), FIVE(5), SIX(6), SEVEN(7), EIGHT(8), NINE(9);

        public final int value;
        private pipeline(int val){
            this.value = val;
        }     
    }

    // Sets limelight’s streaming mode
    public enum stream {
        STANDARD(0),
        PIP_MAIN(1),
        PIP_SECONDARY(2);

        public final int value;
        private stream(int val){
            this.value = val;
        }     
    }

    // Allows users to take snapshots during a match
    public enum snapshot {
        STOP_SNAPSHOTS(0),
        TAKE_SNAPSHOTS(1);

        public final int value;
        private snapshot(int val){
            this.value = val;
        }     
    }
}