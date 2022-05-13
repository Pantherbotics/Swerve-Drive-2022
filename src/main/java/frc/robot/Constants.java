package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

@SuppressWarnings({"unused"})
public class Constants {
    public enum EncoderType {
        Potentiometer, CanCoder
    }
    //--------------------------------------------------------------------------------------------
    //      Notes:
    //-If we get new wheels, edit kWheelDiameterMeters
    //--------------------------------------------------------------------------------------------

    /**
     * The encoder type we are using for the swerve
     * Options: CanCoder, Potentiometer
     */
    public static final EncoderType kEncoderType = EncoderType.Potentiometer;
    //If using the Potentiometer, this specifies its max value so we can get the angle
    public static final double potMax = 3798;

    public static final double neoMaxRPM = 4000;

    //Joysticks
    public static final int pJoyID = 0;

    //Drivetrain
    public static final double angleGearRatio = 1.0; //TODO: Calculate this (We have below, sort of)

    //Checked and verified as of May 1st, 2022
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //4 when new
        public static final double kDriveMotorGearRatio = 2/15D; // 12:30 then 15:45
        public static final double kTurningMotorGearRatio = 0.036; // 12:100 then 18:60
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kITurning = 0.0;
        public static final double kDTurning = 0.0;
    }

    //Checked and verified as of May 1st, 2022
    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(19.5);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(19.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d( kWheelBase / 2,  kTrackWidth / 2),  //Left Front
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),  //Right Front
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  //Right Back
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2)   //Left Back
        );

        //Assume 4000RPM under driving load
        public static final double kPhysicalMaxSpeedMetersPerSecond = (4000/60D) * ModuleConstants.kDriveEncoderRot2Meter; //~2.65988 m/s
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 5; //About 5 given wheelbase and drive speed

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond; //we have about 8.7 ft/s, we don't need to reduce it
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond; //307 degrees per second doesn't need to be reduced
        //These values could be tuned:
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5;
    }

    //Checked and verified as of May 1st, 2022
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 24;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5; //2.5
        //All the following Constants can be tuned:
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0; //1.0
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 2.5; //1.5
        public static final double kPYController = 2.5; //2.5
        public static final double kPThetaController = 1.0; //3

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    //Checked and verified as of May 1st, 2022
    public static final class OIConstants {
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.04; //Higher than average on the controller I'm using
    }
}
