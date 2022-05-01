package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.DriveMode;
import frc.robot.util.SwerveModuleProto;

//TODO ensure all units in this class are correct
@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    public final SwerveModuleProto leftFront;
    public final SwerveModuleProto rightFront;
    public final SwerveModuleProto leftBack;
    public final SwerveModuleProto rightBack;
    public DriveMode mode = DriveMode.FO_SWERVE;

    private final AHRS gyro = new AHRS(I2C.Port.kOnboard); //TODO figure out how to rewire [new AHRS(SPI.Port.kMXP)]
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));


    double debounce = 0.08; //Joystick debounce if sticks don't rest at 0
    double W = 18; //Width of the Robot Chassis
    double L = 18; //Length of the Robot Chassis

    //The offset Angle that the front left wheel would have to adjust in order to rotate the robot clockwise
    // when driving forwards (positively)
    //This value is shifted for the other wheels automatically.
    double rotationAngle = 90 - (Math.atan((L/2) / (W/2)) * 180/Math.PI);

    public Drivetrain(){
        leftFront  = new SwerveModuleProto(1,  -80);
        rightFront = new SwerveModuleProto(2,  155);
        rightBack  = new SwerveModuleProto(3,    0);
        leftBack   = new SwerveModuleProto(4, -160);

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

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), leftFront.getState(), rightFront.getState(), leftBack.getState(), rightBack.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        leftFront.stop();
        rightFront.stop();
        leftBack.stop();
        rightBack.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        leftFront.setDesiredState(desiredStates[0]);
        rightFront.setDesiredState(desiredStates[1]);
        leftBack.setDesiredState(desiredStates[2]);
        rightBack.setDesiredState(desiredStates[3]);
    }
}

