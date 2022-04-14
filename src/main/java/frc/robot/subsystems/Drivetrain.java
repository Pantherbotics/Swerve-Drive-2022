package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Wheel;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
    private final Wheel LeftFront;
    private final Wheel RightFront;
    private final Wheel LeftRear;
    private final Wheel RightRear;
    private final AHRS gyro = new AHRS(I2C.Port.kOnboard);
    public int mode = 4;


    //steering 
    double L = 18; //length
    double W = 18; // Width
    double R = Math.sqrt(L*L+W*W);

    public double targetGyro = 0;
    public double initialGyro = 0;
    public Drivetrain(RobotContainer robotContainer) {
        //Steering (Positive is Clockwise Looking at bottom of robot)
        LeftFront  = new Wheel(1, -75);
        RightFront = new Wheel(2, 165);
        RightRear  = new Wheel(3,   0);
        LeftRear   = new Wheel(4,-150);

        zeroGyro();
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                setGyroTarget();
                initialGyro = getGyroRot() % 360;
            }
        }, 500);

        AtomicBoolean previousTrue = new AtomicBoolean(false);
        Notifier notifier = new Notifier(() -> {
            double steerAxis = robotContainer.pJoy.getRawAxis(4);
            SmartDashboard.putNumber("SteerAxis", steerAxis);
            if (Math.abs(steerAxis) >= 0.05) {
                targetGyro += steerAxis*2D;
                previousTrue.set(true);
            }else if (previousTrue.get()) {
                //targetGyro = getGyroRot();
                previousTrue.set(false);
            }
        });
        notifier.startPeriodic(20/1000D);
    }

    public void zeroGyro() { gyro.reset(); }
    public void setGyroTarget() { targetGyro = getGyroRot() % 360; }
    public double getGyroRot() {
        return gyro.getAngle() % 360;
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
            XL = -XL;
            //Debounce
            if (Math.abs(XL) <= 0.05) { XL = 0; }
            if (Math.abs(YL) <= 0.05) { YL = 0; }

            double gyroErrorDeg = targetGyro - getGyroRot();
            double offset = initialGyro - getGyroRot(); //For Field-Oriented Swerve
            double rotX = gyroErrorDeg/50 + offset/100;//gyroErrorDeg/60;
            double rotY = gyroErrorDeg/50 + offset/100;//gyroErrorDeg/60;

            SmartDashboard.putNumber("GyroInit", initialGyro);
            SmartDashboard.putNumber("Offset", offset);

            //calculate base vectors
            double X1 = XL+rotX;//left Front
            double Y1 = YL+rotY;
            double tAng1 = -Math.atan2(Y1,X1)*180/Math.PI+90;
            double X2 = XL+rotX;//Right Front
            double Y2 = YL-rotY;
            double tAng2 = -Math.atan2(Y2,X2)*180/Math.PI+90;
            double X3 = XL-rotX;//Right Rear
            double Y3 = YL-rotY;
            double tAng3 = -Math.atan2(Y3,X3)*180/Math.PI+90;
            double X4 = XL-rotX;//Left Rear
            double Y4 = YL+rotX;
            double tAng4 = -Math.atan2(Y4,X4)*180/Math.PI+90;
            //Calculate speeds (Divide by sqrt2 so that the speed of x1 y1 is also 1
            double s1 = Math.sqrt(X1*X1+Y1*Y1) / 1.41;
            double s2 = Math.sqrt(X2*X2+Y2*Y2) / 1.41;
            double s3 = Math.sqrt(X3*X3+Y3*Y3) / 1.41;
            double s4 = Math.sqrt(X4*X4+Y4*Y4) / 1.41;
            //Check to see if max speed is <1
            double sMax = Math.max(Math.max(Math.max(s1,s2), s3), s4);

            if (sMax > 1) {
                s1 = s1/sMax;
                s2 = s2/sMax;
                s3 = s3/sMax;
                s4 = s4/sMax;
            }
            //invoke Wheel
            LeftFront.setState(s1, tAng1);
            RightFront.setState(s2,tAng2);
            RightRear.setState(s3, tAng3);
            LeftRear.setState(s4, tAng4);

            //Remove the rotation vector's speed influence
            // Aka have it affect rotation but not speed
        }
    }
}
