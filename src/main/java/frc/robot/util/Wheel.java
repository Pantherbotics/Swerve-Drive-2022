package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

@SuppressWarnings("unused")
public class Wheel extends SwerveModule{
    private final SparkMaxPIDController pidController;

    //Steering stuff
    private final TalonSRX steer;
    private final AnalogInput analogInput;

    //Variable declarations
    double maxRPM = 4000;
    double kP;
    double kI;
    double kD;
    double potMax = 3798;
    double sumError = 0, errorChange = 0, lastError = 0;
    double offset;

    //Constructor Run once when drive is turned on.
    public Wheel(int port, int offset, PID pid) {
        kP = pid.kP; kI = pid.kI; kD = pid.kD;

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

        //Steering Motor setup
        steer = new TalonSRX(port);//set in port ())create talon object
        analogInput = new AnalogInput(port-1);//direction pot

        //CRITICAL: Once the CANcoders arrive, we can use the following methods to use them as sensors
        // Inside the TalonSRX for it's PID loops
        //steer.configRemoteFeedbackFilter(new CANCoder(0), 0);
        //steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);

        // This class is what I've been searching for...
        // RobotController.getBatteryVoltage();
        // RobotController.getVoltage5V();
    }

    //Setter Run each time wheel is updated.
    private double steerTarget = 0;
    public void setState(double speed, double target) {
        steerTarget = target;

        //Custom PID loop
        double error = getModifiedError(target);
        sumError += error * 0.02;
        errorChange = (error-lastError)/0.02;
        double pidOutput = error * kP + kI * sumError + kD * errorChange;
        steer.set(ControlMode.PercentOutput, pidOutput);
        lastError = error;

        //Drive Speed with spark
        speed = speed*maxRPM; //joystick sets speed 0->1.
        pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    /**
     *
     * @return  the unbounded steering error, in radians
     */
    public double getError(double target) {
        double ang = analogInput.getValue(); //analog in on the Rio
        ang = ang*360/potMax + offset + 90; //Convert to compass type heading + offset
        if(ang > 360) { ang = ang - 360; } //correct for offset overshoot.
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

    @Override
    public void update(double velocity, double angle) {
        setState(velocity, angle);
    }

    @Override
    public boolean isAtTarget() {
        return Math.abs(getError(steerTarget)) <= 5;
    }
}
