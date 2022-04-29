package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pantherbotics.libraries.PID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DriveMode;
import frc.robot.util.Wheel;

import java.util.Timer;
import java.util.TimerTask;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    public final Wheel leftFront;
    public final Wheel rightFront;
    public final Wheel leftBack;
    public final Wheel rightBack;
    public DriveMode mode = DriveMode.SWERVE;

    private final AHRS gyro = new AHRS(I2C.Port.kOnboard);

    double W = 18; //Width of the Robot Chassis
    double L = 18; //Length of the Robot Chassis

    //The offset Angle that the front left wheel would have to adjust in order to rotate the robot clockwise
    // when driving forwards (positively)
    //This value is shifted for the other wheels automatically.
    double rotationAngle = 90 - (Math.atan((L/2) / (W/2)) * 180/Math.PI);

    public Drivetrain(){
        //Steering
        PID drivePID = new PID(0.3, 0, 0, 0, 128, 1.0);
        PID steerPID = new PID(0.3, 0, 0, 0, 0, 0.5);

        leftFront  = new Wheel(1,  -80);
        rightFront = new Wheel(2,  155);
        rightBack  = new Wheel(3,    0);
        leftBack   = new Wheel(4, -160);

        //leftFront  = new SwerveModule(1, 1,  2,  3, 0, drivePID, steerPID);
        //rightFront = new SwerveModule(2, 4,  5,  6, 0, drivePID, steerPID);
        //rightBack  = new SwerveModule(3, 7,  8,  9, 0, drivePID, steerPID);
        //leftBack   = new SwerveModule(4, 10,11, 12, 0, drivePID, steerPID);

        //Zero the gyro (which doesn't set it to 0), so then have the target be the current rotation after 100ms
        zeroGyro();
        (new Timer()).schedule(new TimerTask() {
            @Override
            public void run() {
                setTargetRotation(getGyroRot());
                initialRotation = getGyroRot();
            }
        }, 100);
    }

    public void setMode(DriveMode mode) {
        this.mode = mode;
    }

    private double targetRotation = 0;
    private double initialRotation = 0;
    public void setTargetRotation(double targetRotation) {
        this.targetRotation = targetRotation;
    }

    boolean justTurned = false;
    public void runSwerve(double XL, double YL, double XR, double YR) {
        //Code which can be used in the future when we implement rotation stabilization
        if (Math.abs(XR) >= 0.05) {
            justTurned = true;
            targetRotation += XR;
        }else if (justTurned) {
            justTurned = false;
            setTargetRotation(getGyroRot());
        }

        if (mode == DriveMode.FO_SWERVE) {
            //Left Joystick Angle (0 is forwards, 90 is to the right, and 180 is backwards, etc.)
            double joyHeading = getHeading(XL, YL);

            //TODO add rotation stabilization
            //These two variables are for the wheel rotation adjustment for spinning while driving (stabilized rotation)
            double rotTargetError = targetRotation - getGyroRot(); //Should be +-[0, 360)
            double deltaTargetRot2 = rotTargetError / 8D; //Should be +-[0, 45)
            SmartDashboard.putNumber("Rotation Target Error", rotTargetError);
            SmartDashboard.putNumber("Delta Target Rot", deltaTargetRot2);

            //This variable is for wheel joyHeading (Field-Oriented) if we are driving straight (no turning)
            double rotFromInitial = initialRotation - getGyroRot(); //Should be +-[0, 360)
            double heading = joyHeading + rotFromInitial;

            //Wheel Mappings for Vector Variable Names:
            //  1 is Left Front
            //  2 is Right Front
            //  3 is Right Back
            //  4 is Left Back
            //To understand the vector addition look here: https://imgur.com/a/iMfg07P
            double xr = XR * (Math.cos(rotationAngle) / 2D); //For Square Chassis at 45 degrees this is ~0.707/2D
            double yr = XR * (Math.sin(rotationAngle) / 2D); //For Square Chassis at 45 degrees this is ~0.707/2D
            double x = getHeadingX(heading); //cos(heading) but with custom joyHeading angle format
            double y = getHeadingY(heading); //sin(heading) but with custom joyHeading angle format
            double X1 = x + xr;
            double Y1 = y + yr;
            double X2 = x + xr;
            double Y2 = y - yr;
            double X3 = x - xr;
            double Y3 = y - yr;
            double X4 = x - xr;
            double Y4 = y + yr;
            //test

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

    //I spent like half an hour figuring this out, don't try to figure it out just appreciate the results :)
    //0 Degrees is straight forward, 90 degrees is to the right, 180 degrees is backwards, 270 degrees is to the left
    // Aka clockwise degrees and 0 is straight forward on the joystick :)
    private double getHeading(double x, double y) {
        double angle = (360 - ((Math.atan2(y, x)*180/Math.PI) + 180)) - 90;
        if (angle < 0) {
            angle = 270 + (90 - Math.abs(angle));
        }
        return angle;
    }

    //Used to re-obtain the X value from an angle using the custom getHeading()
    private double getHeadingX(double angle) {
        //Ensure values are [0, 360)
        while (angle > 360) { angle -= 360; }
        while (angle < 0) { angle += 360; }

        if (angle >= 0 && angle <= 90) {
            return Math.cos(Math.toRadians(90 - angle));
        }else if (angle >= 90 && angle <= 270) {
            return Math.cos(-Math.toRadians(angle - 90));
        }else if (angle >= 270 && angle <= 360) {
            return -Math.cos(Math.toRadians(270 - angle));
        }
        return 0;
    }
    //Used to re-obtain the Y value from an angle using the custom getHeading()
    private double getHeadingY(double angle) {
        //Ensure values are [0, 360)
        while (angle > 360) { angle -= 360; }
        while (angle < 0) { angle += 360; }

        if (angle >= 0 && angle <= 90) {
            return Math.sin(Math.toRadians(90 - angle));
        }else if (angle >= 90 && angle <= 270) {
            return Math.sin(-Math.toRadians(angle - 90));
        }else if (angle >= 270 && angle <= 360) {
            return -Math.sin(Math.toRadians(270 - angle));
        }
        return 0;
    }

    //Gyroscope
    public double getGyroRot() {
        return gyro.getAngle();
    }

    public void zeroGyro() {
        gyro.reset();
    }
}

