package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.AnalogInput;

@SuppressWarnings("unused")
public class Wheel {
    private final SparkMaxPIDController pidController;

    double maxRPM;
    //Steering stuff
    private final TalonSRX Steer;
    private final AnalogInput Ang;

    //Variable declarations
    double error;
    double kP_steer = 1;
    double potMax;
    double ang;
    double offset;
    
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
        pidController.setP(0.0001);//works well for swerve 0.0001
        pidController.setI(0);
        pidController.setD(.0001);
        pidController.setIZone(0);
        pidController.setFF(.000175);//good for bench, may be increased on the floor
        pidController.setOutputRange(-1, 1);//min and max to motor
        maxRPM = 4000;  //Neo max is 5000, but you need a margin.

        //Steering Motor setup
        Steer = new TalonSRX(port);//set in port ())create talon object
        Ang =new AnalogInput(port-1);//direction pot
        potMax = 3798;
    }

    //Setter Run each time wheel is updated.   
    public void setState(double speed, double Target) {
        //PID Steering  with Talon and external pot 
        ang=Ang.getValue();//analog in on the Rio 
        ang=ang*360 / potMax+offset+90;//Convert to compass type heading + offset
        if(ang>360){ang=ang-360;}//correct for offset overshoot.    
        double error =(Target-ang);
        if (error>180 ) {error=error-360;}//take the shortest path to correct error
        if (error<-180){error=error+360;}
        error=(error/180);//convert to 0=>1 scale 
        error= error*kP_steer;
        if (error>.3){error=.3;}//limit response speed. may be unnecessary  
        if(error<-.3){error=-.3;}
        Steer.set(ControlMode.PercentOutput, error);
        //Drive Speed with spark
        speed=speed*maxRPM;//joystick sets speed 0->1.
        pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }
}
