package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Wheel;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    private final Wheel LeftFront;
    private final Wheel RightFront;
    private final Wheel LeftRear;
    private final Wheel RightRear;
    public int mode = 1;

    //steering 
    double L = 18; //length
    double W = 18; // Width
    double R = Math.sqrt(L*L+W*W);

    public Drivetrain(){
        //Steering
        LeftFront  = new Wheel(1, -80);
        RightFront = new Wheel(2, 160);
        RightRear  = new Wheel(3,   0);
        LeftRear   = new Wheel(4,-170);
    }

    public void setMode(int mode) {
        this.mode = mode;
    }

    public void runSwerve(double XL, double YL, double XR, double YR) {

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
            LeftFront.setState(speed, TargetAng);
            RightFront.setState(speed, TargetAng);
            RightRear.setState(speed, 0);
            LeftRear.setState(speed, 0);
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
            LeftFront.setState(speed, 0);
            RightFront.setState(speed, 0);
            RightRear.setState(speed,-TargetAng);
            LeftRear.setState(speed, -TargetAng);
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
            LeftFront.setState(speed, TargetAng);
            RightFront.setState(speed, TargetAng);
            RightRear.setState(speed, -TargetAng);
            LeftRear.setState(speed, -TargetAng);
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
            LeftFront.setState(s1, tAng1);
            RightFront.setState(s2,tAng2);
            RightRear.setState(s3, tAng3);
            LeftRear.setState(s4, tAng4);
        }
    }
}
