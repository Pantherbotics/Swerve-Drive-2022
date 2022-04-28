package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class Wheel {
    private final SparkMaxPIDController pidController;

    double maxRPM;
    //Steering stuff
    private final TalonSRX Steer;
    private final AnalogInput Ang;

    //Variable declarations
    double error;
    double kP = 1.5;
    double kI = 0.0;
    double kD = 0.0;
    double potMax = 3798;
    double ang;
    double offset;
    double sumError = 0, errorChange = 0, lastError = 0;

    //Constructor Run once when drive is turned on.
    public Wheel(int port, int offset) {
        this.offset= offset;
        //Drive motor
        //Class Declarations
        //Drive Wheel stuff
        CANSparkMax m_motor = new CANSparkMax(port, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        pidController = m_motor.getPIDController();
        // set Drive motor PID coefficients
        pidController.setP(.0001);//works well for swerve
        pidController.setI(0);
        pidController.setD(.0001);
        pidController.setIZone(0);
        pidController.setFF(.000175);//good for bench, may be increased on the floor
        pidController.setOutputRange(-1, 1);//min and max to motor
        maxRPM = 4000;  //Neo max is 5000, but you need a margin.

        //Steering Motor setup
        Steer= new TalonSRX(port);//set in port ())create talon object
        Ang=new AnalogInput(port-1);//direction pot
    }

    //Setter Run each time wheel is updated.
    public void setState(double speed, double target) {
        //PID Steering  with Talon and external pot
        ang=Ang.getValue();//analog in on the Rio
        ang=ang*360/potMax+offset+90;//Convert to compass type heading + offset
        if(ang>360){ang=ang-360;}//correct for offset overshoot.
        double error =(target-ang);
        if (error>180 ) {error=error-360;}//take the shortest path to correct error
        if (error<-180){error=error+360;}
        error=(error/180);//convert to 0=>1 scale
        error= error* kP;
        if (error>.3){error=.3;}//limit response speed. may be unnecessary
        if(error<-.3){error=-.3;}

        //Custom PID loop
        error = getModifiedError(target);
        sumError += error * 0.02;
        errorChange = (error-lastError)/0.02;
        double pidOutput = error * kP + kI * sumError + kD * errorChange;
        Steer.set(ControlMode.PercentOutput, pidOutput);
        lastError = error;

        //Drive Speed with spark
        speed=speed*maxRPM;//joystick sets speed 0->1.
        pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    /**
     *
     * @return  the unbounded steering error, in radians
     */
    public double getError(double target) {
        ang=Ang.getValue();//analog in on the Rio
        ang=ang*360/potMax+offset+90;//Convert to compass type heading + offset
        if(ang>360){ang=ang-360;}//correct for offset overshoot.
        return target - ang;
    }

    /**
     *
     * @return  the steering error bounded to [-pi, pi]
     */
    public double getModifiedError(double target){
        return (boundHalfDegrees(getError(target)))/180;
    }

    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }
}
