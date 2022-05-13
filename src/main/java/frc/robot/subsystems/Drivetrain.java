package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.PID;
import frc.robot.util.SwerveModuleProto;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    public final SwerveModuleProto leftFront;
    public final SwerveModuleProto rightFront;
    public final SwerveModuleProto leftBack;
    public final SwerveModuleProto rightBack;

    private final AHRS gyro = new AHRS(I2C.Port.kOnboard);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));

    public Drivetrain() {
        PID pid = new PID(1.0, 0.0005, 0);
        leftFront  = new SwerveModuleProto(1,  165, pid); //165
        rightFront = new SwerveModuleProto(2,  290, pid); //290
        rightBack  = new SwerveModuleProto(3,   90, pid); //90
        leftBack   = new SwerveModuleProto(4,  -20, pid); //-20

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
        return gyro.getYaw();
        //return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
        //Pose2d curr = odometer.getPoseMeters();
        //return new Pose2d(new Translation2d(curr.getX(), -curr.getY()), curr.getRotation());
    }

    public void resetOdometry(Pose2d pose) {
        DriverStation.reportWarning(pose.toString(), false);
        odometer.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic() {
        //odometer.update(getRotation2d(), leftFront.getState(), rightFront.getState(), leftBack.getState(), rightBack.getState());
        SwerveModuleState lF = leftFront.getState();
        SwerveModuleState rF = rightFront.getState();
        SwerveModuleState rB = rightBack.getState();
        SwerveModuleState lB = leftBack.getState();
        lF = new SwerveModuleState(lF.speedMetersPerSecond, new Rotation2d(lF.angle.getRadians()));
        rF = new SwerveModuleState(rF.speedMetersPerSecond, new Rotation2d(rF.angle.getRadians()));
        rB = new SwerveModuleState(rB.speedMetersPerSecond, new Rotation2d(rB.angle.getRadians()));
        lB = new SwerveModuleState(lB.speedMetersPerSecond, new Rotation2d(lB.angle.getRadians()));

        odometer.update(Rotation2d.fromDegrees(getHeading()), lF, rF, rB, lB);
        //odometer.update(getRotation2d(), rightFront.getState(), leftFront.getState(), rightBack.getState(), leftBack.getState());


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
        rightBack.setDesiredState(desiredStates[2]);
        leftBack.setDesiredState(desiredStates[3]);
    }

    public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //Flip left and right sides, I don't know why
        leftFront.setDesiredStateAuto(desiredStates[1]);
        rightFront.setDesiredStateAuto(desiredStates[0]);
        leftBack.setDesiredStateAuto(desiredStates[3]);
        rightBack.setDesiredStateAuto(desiredStates[2]);
    }
}

