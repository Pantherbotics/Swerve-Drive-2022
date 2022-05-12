package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.DriveMode;
import frc.robot.util.Odometer;
import frc.robot.util.PID;
import frc.robot.util.SwerveModuleProto;

import static frc.robot.util.MathUtils.*;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    //Swerve variables
    public final SwerveModuleProto leftFront, rightFront, leftBack, rightBack;
    public DriveMode mode = DriveMode.FO_SWERVE;

    //Other swerve variables
    private final AHRS gyro = new AHRS(I2C.Port.kOnboard);
    private final Odometer odometer = new Odometer();

    //Initial rotation so we can do field-oriented drive
    public double initialRotation = 0;
    public Drivetrain(){
        //Create the Swerve module objects
        PID steerPID = new PID(1.0, 0.0005, 0);
        //offsetDeg (- is cw, + is ccw)
        leftFront  = new SwerveModuleProto(1, -55, steerPID);
        rightFront = new SwerveModuleProto(2,  45, steerPID);
        rightBack  = new SwerveModuleProto(3, 167, steerPID);
        leftBack   = new SwerveModuleProto(4, -17, steerPID);

        //Zero the gyro after a second (to let it calibrate)
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroGyro();
            } catch (Exception ignored) {}
        }).start();
    }

    //Set the drive mode (mainly driving DriveMode.FO_SWERVE)
    public void setMode(DriveMode mode) {
        this.mode = mode;
    }

    //Get the "pose" or current position of the robot from the odometer
    public Pose2d getPose() {
        return new Pose2d(odometer.x, odometer.y, Rotation2d.fromDegrees(getGyroRot()));
    }

    //Set the odometer to a specified position
    public void resetOdometry(Pose2d pose) {
        DriverStation.reportWarning("Set Odometry: " + pose, false);
        odometer.resetPosition(pose);
    }

    //Update the odometry by calculating the current wheel vectors, the overall odometry vector, then the amount of movement
    private double prevTimeSeconds = -1;
    public void updateOdometry() {
        //Speeds of the wheels [-maxDriveVel, maxDriveVel]
        double s1 = leftFront.getDriveVel();
        double s2 = rightFront.getDriveVel();
        double s3 = rightBack.getDriveVel();
        double s4 = leftBack.getDriveVel();
        //Angles of the wheels [0, 360)
        double a1 = leftFront.getAngle();
        double a2 = rightFront.getAngle();
        double a3 = rightBack.getAngle();
        double a4 = leftBack.getAngle();
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
        double changeY = oX * ModuleConstants.DRIVE_ENCODER_RPM_2_METER_PER_SEC * period;
        double changeX = oY * ModuleConstants.DRIVE_ENCODER_RPM_2_METER_PER_SEC * period;
        odometer.update(changeX, changeY);
    }


    public void runSwerve(double XL, double YL, double XR, double YR) {
        //Update the odometer
        updateOdometry();

        //Add small debounces
        XL = (Math.abs(XL) <= Constants.joyDebounce) ? 0 : XL; YL = (Math.abs(YL) <= Constants.joyDebounce) ? 0 : YL;
        XR = (Math.abs(XR) <= Constants.joyDebounce) ? 0 : XR; YR = (Math.abs(YR) <= Constants.joyDebounce) ? 0 : YR;

        SmartDashboard.putString("Joy Data", "XL: " + round(XL, 2) + " YL: " + round(YL, 2) + " XR: " + round(XR, 2) + " YR: " + round(YR, 2));

        if (mode == DriveMode.FO_SWERVE) {
            //Left Joystick Angle (0 is forwards, 90 is to the right, and 180 is backwards, etc.)
            double joyHeading = getHeading(XL, YL);
            SmartDashboard.putNumber("Heading", joyHeading);
            double speed = Math.sqrt(XL*XL + YL*YL); //[-1, 1] of the speed of the left joystick

            //This variable is for wheel joyHeading (Field-Oriented) if we are driving straight (no turning)
            double rotFromInitial = initialRotation - getGyroRot(); //Should be +-[0, 360)
            double heading = joyHeading + rotFromInitial;

            //Wheel Mappings for Vector Variable Names:
            //  1 is Left Front
            //  2 is Right Front
            //  3 is Right Back
            //  4 is Left Back
            //To understand the vector addition look here: https://imgur.com/a/iMfg07P
            double xr = XR * (Math.cos(Constants.rotationAngle) / 2D); //For Square Chassis at 45 degrees this is ~0.707/2D
            double yr = XR * (Math.sin(Constants.rotationAngle) / 2D); //For Square Chassis at 45 degrees this is ~0.707/2D

            //Rotation Stabilization (NOT WORKING)
            //double xr = (Math.abs(deltaTargetRot)/22.5) * (Math.cos(Math.abs(deltaTargetRot)) / 2D); //For Square Chassis at 45 degrees this is ~0.707/2D
            //double yr = (Math.abs(deltaTargetRot)/22.5) * (Math.sin(Math.abs(deltaTargetRot)) / 2D); //For Square Chassis at 45 degrees this is ~0.707/2D

            double x = getHeadingX(heading); //cos(heading) but with custom joyHeading angle format
            double y = getHeadingY(heading); //sin(heading) but with custom joyHeading angle format
            double X1 = x*speed + xr;
            double Y1 = y*speed + yr;
            double X2 = x*speed + xr;
            double Y2 = y*speed - yr;
            double X3 = x*speed - xr;
            double Y3 = y*speed - yr;
            double X4 = x*speed - xr;
            double Y4 = y*speed + yr;
            //SmartDashboard.putString("Data", "X: " + x + " Y: " + y);
            //SmartDashboard.putString("Data2", "RX: " + xr + " RY: " + yr);

            double speedMax = Math.sqrt(xr*xr + (1+yr)*(1+yr)); //The largest possible speed from vectors
            leftFront.setState(Math.sqrt(X1*X1+Y1*Y1)/speedMax, getHeading(X1, Y1));
            rightFront.setState(Math.sqrt(X2*X2+Y2*Y2)/speedMax, getHeading(X2, Y2));
            rightBack.setState(Math.sqrt(X3*X3+Y3*Y3)/speedMax, getHeading(X3, Y3));
            leftBack.setState(Math.sqrt(X4*X4+Y4*Y4)/speedMax, getHeading(X4, Y4));

            //Update Wheels
            //leftFront.updateModule((Math.sqrt(X1*X1+Y1*Y1)/speedMax), getHeading(X1, Y1));
            //rightFront.updateModule((Math.sqrt(X2*X2+Y2*Y2)/speedMax), getHeading(X2, Y2));
            //rightBack.updateModule((Math.sqrt(X3*X3+Y3*Y3)/speedMax), getHeading(X3, Y3));
            //leftBack.updateModule((Math.sqrt(X4*X4+Y4*Y4)/speedMax), getHeading(X4, Y4));
        }

        //TODO add Regular SWERVE and GS_SWERVE modes after completing FO_SWERVE

        //Car mode. Front wheel only steering.   Arcade control
        if (mode == DriveMode.CAR) {
            double speed = (YR*YR);//square the speed but keep the sign so it can reverse
            if(YR < 0){ speed = -speed; }
            if (speed > 1){ speed = 1; }
            if (speed < -1){ speed = -1; }

            //Calculate Steering Angle
            double TargetAng = (XR)*90;
            //TargetAng= Math.abs(TargetAng*180/3.14159+180);//convert to degrees
            //invoke Wheel
            leftFront.setState(speed, TargetAng);
            rightFront.setState(speed, TargetAng);
            rightBack.setState(speed, 0);
            leftBack.setState(speed, 0);
        }

        //Boat mode.  Rear steering.   Arcade control
        if (mode == DriveMode.BOAT) {
            //Right stick speed
            double speed = (YR*YR);//square the speed but keep the sign so it can reverse
            if(YR < 0){ speed = -speed; }
            if (speed > 1){ speed = 1; }
            if (speed < -1){ speed = -1; }

            //Calculate Steering Angle
            double TargetAng = (XR)*90;
            //TargetAng= Math.abs(TargetAng*180/3.14159+180);//convert to degrees
            //invoke Wheel
            leftFront.setState(speed, 0);
            rightFront.setState(speed, 0);
            rightBack.setState(speed,-TargetAng);
            leftBack.setState(speed, -TargetAng);
        }
    }

    //Gyroscope
    public double getGyroRot() {
        return gyro.getYaw();
    }

    public void zeroGyro() {
        gyro.reset();
    }
}

