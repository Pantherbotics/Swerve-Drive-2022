package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.*;
import lombok.Getter;
import lombok.Setter;

import static frc.robot.util.MathUtils.getHeadingX;
import static frc.robot.util.MathUtils.getHeadingY;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    @Getter private final SwerveModule leftFront, rightFront, rightBack, leftBack;
    private final SwerveModule[] modules;
    private DriveMode mode = DriveMode.FO_SWERVE;

    @Getter private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Odometer odometer = new Odometer(); //Custom odometer that works for Holonomic Swerve

    @Getter @Setter private double limelightYaw = 0.0;
    @Getter @Setter private boolean lockDriveWhileTargeting = false;
    public Drivetrain() {
        //Positive is CCW, Negative is CW
        leftFront  = new SwerveModuleProto(1,  70); //130
        rightFront = new SwerveModuleProto(2,  225+52);
        rightBack  = new SwerveModuleProto(3,  350-163);
        leftBack   = new SwerveModuleProto(4,  163+108);
        modules = new SwerveModule[] {leftFront, rightFront, rightBack, leftBack};

        //Zero the gyro after 1 second while it calibrates
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception ignored) {}
        }).start();
    }

    //Zero the heading of the gyro (Sets to 0)
    public void zeroHeading() {
        gyro.reset();
    }

    public double autoGyroInit = 0;
    /**
     * Get the rotation of the robot (positive CCW, negative CW)
     * @return the current heading of the robot in degrees [-180, 180]
     */
    public double getHeading() {
        //getYaw for flat mounted gyro
        //getRoll for vertically mounted gyro (where the f**k did you place the rio)

        return MathUtils.boundHalfDegrees(-gyro.getRoll() + autoGyroInit - 90);
        //return -gyro.getYaw();
    }

    /**
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return new Pose2d(odometer.getPoseMeters(), Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Reset the odometer to the specified pose
     * @param pose The new pose
     */
    public void resetOdometry(Rotation2d rotation, Pose2d pose) {
        //When the auto starts it will reset the odometry. If the robot's rotation isn't 0 at the start, configure the gyro
        // to report correct values for the rest of the match.
        autoGyroInit = rotation.getDegrees();
        odometer.resetPosition(pose);
    }

    //Update the odometry by calculating the current wheel vectors, the overall odometry vector, then the amount of movement
    private double prevTimeSeconds = -1;
    public void updateOdometry() {
        //Speeds of the wheels in meters per second
        double s1 = leftFront.getDriveVelocity();
        double s2 = rightFront.getDriveVelocity();
        double s3 = rightBack.getDriveVelocity();
        double s4 = leftBack.getDriveVelocity();
        //Angles of the wheels [0, 360)
        double heading = getHeading();
        double a1 = leftFront.getAngle() + heading;
        double a2 = rightFront.getAngle() + heading;
        double a3 = rightBack.getAngle() + heading;
        double a4 = leftBack.getAngle() + heading;
        //The vector components of the wheels, based on their current values
        double X1 = getHeadingX(a1) * s1;
        double Y1 = getHeadingY(a1) * s1;
        double X2 = getHeadingX(a2) * s2;
        double Y2 = getHeadingY(a2) * s2;
        double X3 = getHeadingX(a3) * s3;
        double Y3 = getHeadingY(a3) * s3;
        double X4 = getHeadingX(a4) * s4;
        double Y4 = getHeadingY(a4) * s4;

        //Calculate the odometry vector components [-maxDriveVel, maxDriveVel]
        double oX = (X1 + X2 + X3 + X4) / 4D;
        double oY = (Y1 + Y2 + Y3 + Y4) / 4D;
        //SmartDashboard.putString("Odo Data", "oX: " + roundStr(oX, 3) + " oY: " + roundStr(oY, 3));

        //Calculate the period in seconds since last update
        double currTimeSec = WPIUtilJNI.now() * 1.0e-6;
        double period = prevTimeSeconds >= 0 ? currTimeSec - prevTimeSeconds : 0.0; prevTimeSeconds = currTimeSec;

        //oX is the distance traveled in meters in a second, then multiplied by the period
        double changeY = oX * period;
        double changeX = oY * period;
        odometer.update(changeX, changeY);
    }


    @Override
    public void periodic() {
        //Update the odometry, using our own vector-based odometry for Holonomic Swerve
        updateOdometry();
        //TalonSRX srx = ((SwerveModuleProto) leftFront).getSteer();
        //SmartDashboard.putNumber("LF Steer Pos", srx.getSelectedSensorPosition());
        //double output = srx.getStatorCurrent();
        //SmartDashboard.putNumber("LF Steer Curr", output);

        CANSparkMax spark = ((SwerveModuleProto) leftFront).getDrive();
        SmartDashboard.putNumber("LF Drive Pos", spark.getEncoder().getPosition());
        double output = spark.getOutputCurrent();
        SmartDashboard.putNumber("LF Drive Curr", output);
        if (output >= 30D) {
            spark.getEncoder().setPosition(0);
        }
    }

    /**
     * Set the drive mode
     * @param mode The new mode
     */
    public void setMode(DriveMode mode) {
        this.mode = mode;
    }

    /**
     * @return The drive mode the swerve is operating in
     */
    public DriveMode getMode() {
        return mode;
    }

    /**
     * Invoke stop() on all modules so the robot stops
     */
    public void stopModules() {
        leftFront.stop();
        rightFront.stop();
        leftBack.stop();
        rightBack.stop();
    }

    /**
     * @param desiredStates The states to set the modules to, in the order specified in kinematics
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for (int i = 0; i < 4; i++) {
            //Optimize States (Both Teleop and Auto gives unoptimized)
            modules[i].setDesiredState(SwerveModuleState.optimize(desiredStates[i], Rotation2d.fromDegrees(modules[i].getAngle())));
        }
    }
}

