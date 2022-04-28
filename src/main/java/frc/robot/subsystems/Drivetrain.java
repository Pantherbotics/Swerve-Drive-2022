package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DriveMode;
import frc.robot.util.PID;
import frc.robot.util.Wheel;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    private final Wheel leftFront;
    private final Wheel rightFront;
    private final Wheel leftBack;
    private final Wheel rightBack;
    public DriveMode mode = DriveMode.SWERVE;

    double W = 18; //Width of the Robot Chassis
    double L = 18; //Length of the Robot Chassis

    //The offset Angle that the front right wheel would have to adjust in order to rotate the robot counterclockwise
    // when driving forwards (positively)
    //Other wheels are adjusted based on this value.
    double rotationAngle = Math.atan((L/2) / (W/2)) - 90; //TODO: implement rotating

    public Drivetrain(){
        //Steering
        PID drivePID = new PID(0.3, 0, 0, 0, 128, 1.0);
        PID steerPID = new PID(0.3, 0, 0, 0, 128, 0.5);


        leftFront  = new Wheel(1,  -80);
        rightFront = new Wheel(2,  160);
        rightBack  = new Wheel(3,    0);
        leftBack   = new Wheel(4, -170);

        //leftFront  = new SwerveModule(1, 1,  2,  3, 0, drivePID, steerPID);
        //rightFront = new SwerveModule(2, 4,  5,  6, 0, drivePID, steerPID);
        //rightBack  = new SwerveModule(3, 7,  8,  9, 0, drivePID, steerPID);
        //leftBack   = new SwerveModule(4, 10,11, 12, 0, drivePID, steerPID);
    }

    public void setMode(DriveMode mode) {
        this.mode = mode;
    }

    double maxRPM = 5000;
    double rp100ms = ((maxRPM / 60D) / 10D); //Max Revolutions per 100ms
    public void runSwerve(double XL, double YL, double XR, double YR) {
        if (mode == DriveMode.SWERVE) {
            //Left Joystick Angle (0 is forwards, 90 is to the right, and 180 is backwards, etc.)
            double heading = getHeading(XL, YL);
            double speed = Math.sqrt(XL*XL + YL*YL) / 1.41421356; //A 0-1 Value for Speed
            double velocity = (rp100ms * speed) * 2048; //Max Revolutions * Speed Scalar * Ticks

            //Update Wheels
            //leftFront.updateModule(velocity, heading);
            //rightFront.updateModule(velocity, heading);
            //rightBack.updateModule(velocity, heading);
            //leftBack.updateModule(velocity, heading);
            leftFront.setState(speed, heading);
            rightFront.setState(speed, heading);
            rightBack.setState(speed, heading);
            leftBack.setState(speed, heading);
        }


        /*
        //Car mode. Front wheel only steering.   Arcade control
        if (mode == 1) {
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
        if (mode == 2) {
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

        //Snake mode.  Front and Rear steering.   Arcade control
        if (mode==3) {
            //Right stick speed
            double speed = (YR*YR);//square the speed but keep the sign so it can reverse
            if (YR < 0) { speed = -speed; }
            if (speed > 1) { speed = 1; }
            if (speed < -1) { speed = -1; }

            //Calculate Steering Angle
            if (XR < 0) { XR = -XR*XR; }
            else{ XR = XR*XR; }
            double TargetAng = (XR)*90;

            //invoke Wheel
            leftFront.setState(speed, TargetAng);
            rightFront.setState(speed, TargetAng);
            rightBack.setState(speed, -TargetAng);
            leftBack.setState(speed, -TargetAng);
        }

        //Full Swerve Mode Right stick strafes left stick rotates.
        if (mode == 4) {
            //define the rotation vector.
            double rotX = XL/1.41; //R/sqrt2
            double rotY = XL/1.41;

            //calculate base vectors
            double X1 = XR+rotX;//left Front
            double Y1 = YR+rotY;
            double tAng1 = -Math.atan2(Y1,X1)*180/Math.PI+90;
            double X2 = XR+rotX;//Right Front
            double Y2 = YR-rotY;
            double tAng2 = -Math.atan2(Y2,X2)*180/Math.PI+90;
            double X3 = XR-rotX;//Right Rear
            double Y3 = YR-rotY;
            double tAng3 = -Math.atan2(Y3,X3)*180/Math.PI+90;
            double X4 = XR-rotX;//Left Rear
            double Y4 = YR+rotX;
            double tAng4 = -Math.atan2(Y4,X4)*180/Math.PI+90;
            //Calculate speeds
            double s1 = Math.sqrt(X1*X1+Y1*Y1);
            double s2 = Math.sqrt(X2*X2+Y2*Y2);
            double s3 = Math.sqrt(X3*X3+Y3*Y3);
            double s4 = Math.sqrt(X4*X4+Y4*Y4);
            //Check to see if max speed is <1
            double sMax = Math.max(s1,s2);
            sMax = Math.max(sMax,s3);
            sMax = Math.max(sMax,s4);

            if(sMax > 1) {
                s1 = s1/sMax;
                s2 = s2/sMax;
                s3 = s3/sMax;
                s4 = s4/sMax;
            } else{ //rescale to the square of the largest speed.
                s1 = s1*sMax;
                s2 = s2*sMax;
                s3 = s3*sMax;
                s4 = s4*sMax;
            }
            //invoke Wheel
            leftFront.setState(s1, tAng1);
            rightFront.setState(s2,tAng2);
            rightBack.setState(s3, tAng3);
            leftBack.setState(s4, tAng4);
        }
        */
    }

    //I spent like half an hour figuring this out, don't try to figure it out just appreciate the results :)
    //0 Degrees is straight forward, 90 degrees is to the right, 180 degrees is backwards, 270 degrees is to the left
    // Aka clockwise degrees and 0 is straight forward on the joystick :)
    private static double getHeading(double x, double y) {
        double angle = (360 - ((Math.atan2(y, x)*180/Math.PI) + 180)) - 90;
        if (angle < 0) {
            angle = 270 + (90 - Math.abs(angle));
        }
        return angle;
    }
}

