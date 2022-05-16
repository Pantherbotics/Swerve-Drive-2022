package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.MathUtils;
import frc.robot.util.Odometer;
import frc.robot.util.PID;
import frc.robot.util.SwerveModuleProto;

import static frc.robot.util.MathUtils.*;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    public final SwerveModuleProto leftFront;
    public final SwerveModuleProto rightFront;
    public final SwerveModuleProto leftBack;
    public final SwerveModuleProto rightBack;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    //private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));
    private final Odometer odometer = new Odometer();

    public Drivetrain() {
        PID pid = new PID(1.0, 0.0005, 0);
        int offset = 180;
        leftFront  = new SwerveModuleProto(1,  -55+offset, pid); //165
        rightFront = new SwerveModuleProto(2,   45+offset, pid); //290
        rightBack  = new SwerveModuleProto(3,  167+offset, pid); //90
        leftBack   = new SwerveModuleProto(4,  -17+offset, pid); //-20

        //leftFront  = new SwerveModule(1, 1,  2,  3, 0, drivePID, steerPID);
        //rightFront = new SwerveModule(2, 4,  5,  6, 0, drivePID, steerPID);
        //rightBack  = new SwerveModule(3, 7,  8,  9, 0, drivePID, steerPID);
        //leftBack   = new SwerveModule(4, 10,11, 12, 0, drivePID, steerPID);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception ignored) {}
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }


    /**
     * Returns the current heading of the robot in degrees [-180, 180]
     * TODO The examples return it in [0, 360), experiment with this later
     *
     * The WPI trajectory/spline creation specifies rotation of the robot from [-pi, pi] positive ccw
     */
    public double getHeading() {
        //return MathUtils.restrictAngle(-gyro.getAngle());

        return -gyro.getYaw();
        //return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return new Pose2d(odometer.getPoseMeters(), Rotation2d.fromDegrees(getHeading()));
        //Pose2d curr = odometer.getPoseMeters();
        //return new Pose2d(new Translation2d(curr.getX(), -curr.getY()), curr.getRotation());
    }

    public void resetOdometry(Pose2d pose) {
        DriverStation.reportWarning(pose.toString(), false);
        //odometer.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
        odometer.resetPosition(pose);
    }

    //Update the odometry by calculating the current wheel vectors, the overall odometry vector, then the amount of movement
    private double prevTimeSeconds = -1;
    public void updateOdometry() {
        //Speeds of the wheels [-maxDriveVel, maxDriveVel]
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
        SmartDashboard.putString("Odo Data", "oX: " + roundStr(oX, 3) + " oY: " + roundStr(oY, 3));
        //double oHeading = getHeading(oX, oY);
        //double oSpeed = Math.sqrt(oX*oX + oY*oY);
        //oX = getHeadingX(oHeading+odoR) * oSpeed;
        //oY = getHeadingY(oHeading+odoR) * oSpeed;

        //Calculate the period in seconds since last update
        double currTimeSec = WPIUtilJNI.now() * 1.0e-6;
        double period = prevTimeSeconds >= 0 ? currTimeSec - prevTimeSeconds : 0.0; prevTimeSeconds = currTimeSec;

        //oX * DRIVE_VEL_TO_METERS_PER_SECOND is the distance traveled in meters in a second, then multiplied by the period
        double changeY = oX /* * Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec */ * period;
        double changeX = oY /* * Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec */ * period;
        odometer.update(changeX, changeY);
    }


    @Override
    public void periodic() {
        //odometer.update(getRotation2d(), leftFront.getState(), rightFront.getState(), leftBack.getState(), rightBack.getState());
        SwerveModuleState lF = leftFront.getState();
        SwerveModuleState rF = rightFront.getState();
        SwerveModuleState rB = rightBack.getState();
        SwerveModuleState lB = leftBack.getState();
        //lF = new SwerveModuleState(lF.speedMetersPerSecond, new Rotation2d(lF.angle.getRadians()));
        //rF = new SwerveModuleState(rF.speedMetersPerSecond, new Rotation2d(rF.angle.getRadians()));
        //rB = new SwerveModuleState(rB.speedMetersPerSecond, new Rotation2d(rB.angle.getRadians()));
        //lB = new SwerveModuleState(lB.speedMetersPerSecond, new Rotation2d(lB.angle.getRadians()));


        //odometer.update(Rotation2d.fromDegrees(getHeading()), lF, rF, rB, lB);
        updateOdometry();

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", "X: " + round(getPose().getTranslation().getX(),3) + " Y: " + round(getPose().getTranslation().getY(),3));
    }

    public void stopModules() {
        leftFront.stop();
        rightFront.stop();
        leftBack.stop();
        rightBack.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        leftFront.setDesiredState(desiredStates[0], true);
        rightFront.setDesiredState(desiredStates[1], true);
        rightBack.setDesiredState(desiredStates[2], true);
        leftBack.setDesiredState(desiredStates[3], true);
    }

    public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //Flip left and right sides, I don't know why
        //leftFront.setDesiredStateAuto(desiredStates[1]);
        //rightFront.setDesiredStateAuto(desiredStates[0]);
        //leftBack.setDesiredStateAuto(desiredStates[3]);
        //rightBack.setDesiredStateAuto(desiredStates[2]);

        leftFront.setDesiredStateAuto(desiredStates[0]);
        rightFront.setDesiredStateAuto(desiredStates[1]);
        rightBack.setDesiredStateAuto(desiredStates[2]);
        leftBack.setDesiredStateAuto(desiredStates[3]);
    }
}

