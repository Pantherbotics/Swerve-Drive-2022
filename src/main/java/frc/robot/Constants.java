package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

@SuppressWarnings({"unused", "HungarianNotation"})
public class Constants {
    public enum EncoderType {
        Potentiometer, CanCoder
    }

    //Joysticks
    public static final int pJoyID = 0;
    public static final double joyDebounce = 0.0; //0.08;

    /**
     * The encoder type we are using for the swerve
     * Options: CanCoder, Potentiometer
     */
    public static final EncoderType kEncoderType = EncoderType.CanCoder;
    //If using the Potentiometer, this specifies its max value so we can get the angle
    public static final double potMax = 3798;

    //Drivetrain
    private static final double W = 18; //Width of the Robot Chassis
    private static final double L = 18; //Length of the Robot Chassis
    public static final int maxDriveVel = 4000; //Max velocity of the drivetrain in its units

    //The offset Angle that the front left wheel would have to adjust in order to rotate the robot clockwise
    // when driving forwards (positively)
    //This value is shifted for the other wheels automatically.
    public static final double rotationAngle = 90 - Math.toDegrees(Math.atan((L/2) / (W/2))); //For square chassis this is 45 degrees

    public static final double angleGearRatio = 1.0; //TODO: Calculate this

    public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4); //4 when new
        public static final double DRIVE_MOTOR_GEAR_RATIO = 2/15D; // 12:30 then 15:45
        //public static final double TURNING_MOTOR_GEAR_RATIO = 0.036; // 12:100 then 18:60
        public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        //public static final double TURNING_ENCODER_ROT_2_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        //public static final double TURNING_ENCODER_RPM_2_RAD_PER_SEC = TURNING_ENCODER_ROT_2_RAD / 60;
        //public static final double P_TURNING = 0.5;
        //public static final double I_TURNING = 0.0;
        //public static final double D_TURNING = 0.0;
    }

    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.5);
        // Distance between front and back wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(19.5);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
        );

        //Assume 4000RPM under driving load
        //public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = (4000/60D) * ModuleConstants.DRIVE_ENCODER_ROT_2_METER; //~2.65988 m/s
        //public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 5; //About 5 given wheelbase and drive speed

        //public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND; //we have about 8.7 ft/s, we don't need to reduce it
        //public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND; //307 degrees per second doesn't need to be reduced
        ////These values could be tuned:
        //public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 1.5;
        //public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 1.5;
    }
}
