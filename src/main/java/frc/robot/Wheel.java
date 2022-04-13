package frc.robot;
//Imports
//WPI Rio Stuff
import edu.wpi.first.wpilibj.AnalogInput;
//Phoenix Stuff
import com.ctre.phoenix.motorcontrol.can.*; 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;//default is coast.  
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
//import com.ctre.phoenix.motorcontrol.DemandType;
//Rev Stuff
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;//required to define burshed or brushless
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;//not needed but may be used for odometry.
import com.revrobotics.CANPIDController;


public class Wheel {

   //Class Declarations
    //Drive Wheel stuff
    private CANSparkMax m_motor;
    private CANPIDController m_pidController;   

    double maxRPM;
    //Steering stuff
    private TalonSRX  Steer;
    private AnalogInput Ang;

   //Variable declarations 
    double error;
    double kP_steer; 
    double potMax;
    double ang;
    double offset;
    
    //Constructor Run once when drive is turned on.  
    public  Wheel(int port, int offset)
    {
        this.offset= offset;
        //Drive motor
        m_motor= new CANSparkMax(port, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_pidController = m_motor.getPIDController();
            // set Drive motor PID coefficients
        m_pidController.setP(.0001);//works well for swerve
        m_pidController.setI(0);
        m_pidController.setD(.0001);
        m_pidController.setIZone(0);
        m_pidController.setFF(.000175);//good for bench, may be increased on the floor
        m_pidController.setOutputRange(-1, 1);//min and max to motor
        maxRPM = 4000;  //Neo max is 5000, but you need a margin.  

        //Steering Motor setup
        Steer= new TalonSRX(port);//set in port ())create talon object
        Ang=new AnalogInput(port-1);//direction pot
        potMax = 3798;
        kP_steer=1;
        
    }
    //Setter Run each time wheel is updated.   
    public void setState(double speed, double Target)
    { 
        //PID Steering  with Talon and external pot 
        ang=Ang.getValue();//analog in on the Rio 
        ang=ang*360/potMax+offset+90;//Convert to compass type heading + offset
        if(ang>360){ang=ang-360;}//correct for offset overshoot.    
        double error =(Target-ang);
        if (error>180 ) {error=error-360;}//take the shortest path to correct error
        if (error<-180){error=error+360;}
        error=(error/180);//convert to 0=>1 scale 
        error= error*kP_steer;
        if (error>.3){error=.3;}//limit response speed. may be unnecessary  
        if(error<-.3){error=-.3;}
        Steer.set(ControlMode.PercentOutput,error );
        //Drive Speed with spark
        speed=speed*maxRPM;//joystic sets speed 0->1.    
        m_pidController.setReference(speed, ControlType.kVelocity);
    }
}
