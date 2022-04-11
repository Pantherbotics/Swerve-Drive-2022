package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

@SuppressWarnings("unused")
public class OI{

    public Joystick stick = new Joystick(Constants.kJoyStick);
    public OI(){}

    public double getLeftAngle(){
        return stick.getDirectionDegrees();
    }

    public double getLeftJoystickAngle(){
        return Math.toDegrees(Math.atan2(stick.getRawAxis(0), -stick.getRawAxis(1)));
    }

    public double getLeftXAxis(){
        return Math.abs(stick.getRawAxis(0)) > 0.1 ? stick.getRawAxis(0) : 0;
    }

    public double getLeftYAxis(){
        /*
        double val = ((-stick.getRawAxis(1)) + Constants.kLeftYOffset) * (1.0/(1.0-Constants.kLeftYOffset));
        val = val > 1.0 ? 1.0 : val;
        val = val < -1.0 ? -1.0 : val;
        val = Math.abs(val) < .05 ? 0 : val;
        return val;*/
        return Math.abs(stick.getRawAxis(1)) > 0.1 ? stick.getRawAxis(1) : 0;
    }

    public double getRightXAxis(){

        if(stick.getRawButton(1))
            return (Math.abs(stick.getRawAxis(2)) > 0.1 ? stick.getRawAxis(2) : 0) * 0.25;
        else
            return 0;
    }

    public double getLeftMagnitude(){
        return Math.hypot(stick.getRawAxis(1), stick.getRawAxis(0));
    }
}
