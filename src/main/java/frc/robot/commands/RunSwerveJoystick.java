package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DriveMode;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static frc.robot.util.MathUtils.powAxis;

@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class RunSwerveJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick joystick;
    private final Supplier<Double> speedChooser;
    private final Supplier<DriveMode> driveMode;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public RunSwerveJoystick(Drivetrain drivetrain, Joystick joystick, Supplier<Double> speedChooser, Supplier<DriveMode> driveMode) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.speedChooser = speedChooser;
        this.driveMode = driveMode;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (driveMode.get() == DriveMode.FO_SWERVE) {
            runSwerve(true);
        }else if (driveMode.get() == DriveMode.SWERVE) {
            runSwerve(false);
        }else if (driveMode.get() == DriveMode.BOAT) {
            runBoat();
        }else if (drivetrain.getMode() == DriveMode.CAR) {
            runCar();
        }else if (drivetrain.getMode() == DriveMode.WESTCOAST) {
            runWestCoast();
        }else if (drivetrain.getMode() == DriveMode.TANK) {
            runTank();
        }
    }

    private void runSwerve(boolean fieldOriented) {
        // 1. Get real-time joystick inputs, converted to work with Swerve and WPI
        //These adjustments for Swerve were experimentally determined... they work
        double xSpeed = -powAxis(getYL(), OIConstants.driverEXP) * speedChooser.get();
        double ySpeed = -powAxis(getXL(), OIConstants.driverEXP) * speedChooser.get();
        double turningSpeed = -getXR() * (speedChooser.get()/2D);

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(drivetrain.getHeading()));
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 6. Output each module states to wheels
        drivetrain.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    private void runBoat() {
        double YL = -getYL();
        double XR = getXR();

        //Right stick speed
        double speed = (YL*YL);//square the speed but keep the sign so it can reverse
        if (YL < 0) { speed = -speed; }
        if (speed > 1) { speed = 1; }
        if (speed < -1) { speed = -1; }
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; //Scale it up to m/s

        //Calculate Steering Angle
        double TargetAng = (XR)*90;
        SwerveModuleState lF = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));
        SwerveModuleState rF = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));
        SwerveModuleState rB = new SwerveModuleState(speed, Rotation2d.fromDegrees(-TargetAng));
        SwerveModuleState lB = new SwerveModuleState(speed, Rotation2d.fromDegrees(-TargetAng));

        drivetrain.setModuleStates(new SwerveModuleState[] {lF, rF, rB, lB});
    }

    private void runCar() {
        double YL = -getYL();
        double XR = -getXR();

        double speed = (YL*YL);//square the speed but keep the sign so it can reverse
        if(YL < 0){ speed = -speed; }
        if (speed > 1){ speed = 1; }
        if (speed < -1){ speed = -1; }
        speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond; //Scale it up to m/s

        //Calculate Steering Angle
        double TargetAng = (XR)*90;

        SwerveModuleState lF = new SwerveModuleState(speed, Rotation2d.fromDegrees(TargetAng));
        SwerveModuleState rF = new SwerveModuleState(speed, Rotation2d.fromDegrees(TargetAng));
        SwerveModuleState rB = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));
        SwerveModuleState lB = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));

        drivetrain.setModuleStates(new SwerveModuleState[] {lF, rF, rB, lB});
    }

    private void runWestCoast() {
        double YL = getYL();
        double XR = getXR();

        double left = (XR - YL) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double right = (-XR - YL)  * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        SwerveModuleState lF = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        SwerveModuleState rF = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState rB = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState lB = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        drivetrain.setModuleStates(new SwerveModuleState[] {lF, rF, rB, lB});
    }

    private void runTank() {
        double YL = -getYL();
        double YR = -getYR();

        double left = YL * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double right = YR * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        SwerveModuleState lF = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        SwerveModuleState rF = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState rB = new SwerveModuleState(right, Rotation2d.fromDegrees(0));
        SwerveModuleState lB = new SwerveModuleState(left, Rotation2d.fromDegrees(0));
        drivetrain.setModuleStates(new SwerveModuleState[] {lF, rF, rB, lB});
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double getXL() {
        return joystick.getRawAxis(OIConstants.kDriverXL);
    }

    private double getYL() {
        return joystick.getRawAxis(OIConstants.kDriverYL);
    }

    private double getXR() {
        return joystick.getRawAxis(OIConstants.kDriverXR);
    }

    private double getYR() {
        return joystick.getRawAxis(OIConstants.kDriverYR);
    }
}
