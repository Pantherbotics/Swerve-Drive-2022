package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import org.jetbrains.annotations.Nullable;

@SuppressWarnings("unused")
public class SwerveModuleProto extends SwerveModule {
    //Drive stuff
    private final CANSparkMax drive;
    private final SparkMaxPIDController pidController;

    //Steering stuff
    private final TalonSRX steer;
    @Nullable private final AnalogInput analogInput;
    @Nullable private final CANCoder canCoder;

    //Variable declarations
    double kP;
    double kI;
    double kD;
    double potMax = 3798;
    double sumError = 0, errorChange = 0, lastError = 0;
    double offsetDeg;

    /**
     * @param id ID of the module's motors
     * @param offsetDeg Offset degrees of the module
     * @param pid PID constants for the module steering
     */
    public SwerveModuleProto(int id, int offsetDeg, PID pid) {
        kP = pid.kP; kI = pid.kI; kD = pid.kD;

        this.offsetDeg = offsetDeg;
        //Drive motor
        //Class Declarations
        //Drive Wheel stuff
        drive = new CANSparkMax(id, MotorType.kBrushless);
        drive.restoreFactoryDefaults();
        pidController = drive.getPIDController();
        // set Drive motor PID coefficients
        pidController.setP(.0001);
        pidController.setI(0);
        pidController.setD(.0001);
        pidController.setIZone(0);
        pidController.setFF(.000175);//good for bench, may be increased on the floor
        pidController.setOutputRange(-1, 1);//min and max to motor

        //Steering Motor setup
        steer = new TalonSRX(id);//set in id ())create talon object

        if (Constants.kEncoderType == Constants.EncoderType.CanCoder) {
            canCoder = new CANCoder(id + 4);
            canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            canCoder.setPositionToAbsolute();
            steer.configRemoteFeedbackFilter(canCoder, 0);
            steer.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
            steer.config_kP(0, 0.2);
            steer.config_kI(0, 0.0);
            steer.config_kD(0, 0.0);
            steer.config_kF(0, 0.0);

            analogInput = null;
        }else {
            analogInput = new AnalogInput(id - 1);
            canCoder = null;
        }


        //CRITICAL: Once the CANcoders arrive, we can use the following methods to use them as sensors
        // Inside the TalonSRX for it's PID loops
        //steer.configRemoteFeedbackFilter(new CANCoder(0), 0);
        //steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);

        // This class is what I've been searching for...
        // RobotController.getBatteryVoltage();
        // RobotController.getVoltage5V(); //Helpful to interpret analog inputs when source is not 5V
    }

    public double getDriveVel() {
        return drive.getEncoder().getVelocity();
    }

    //Setter Run each time wheel is updated.
    private double steerTarget = 0;

    /**
     * @param speed Percent Output in [-1, 1]
     * @param target Target Angle in Degrees [0, 360)
     */
    public void setState(double speed, double target) {
        steerTarget = target;

        //Custom PID loop
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            canCoder.setPosition(target + offsetDeg);
        }else {
            double error = getModifiedError(target);
            sumError += error * 0.02;
            errorChange = (error-lastError)/0.02;
            double pidOutput = error * kP + kI * sumError + kD * errorChange;
            steer.set(ControlMode.PercentOutput, pidOutput);
            lastError = error;
        }

        //Drive Speed with spark
        speed = speed * Constants.maxDriveVel; //joystick sets speed 0->1.
        pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * @return the current angle of the module's wheel
     */
    public double getAngle() {
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            return canCoder.getAbsolutePosition() + offsetDeg;
        }else if (analogInput != null) {
            double ang = analogInput.getValue(); //analog in on the Rio
            ang = ang*360/potMax + offsetDeg + 90; //Convert to compass type heading + offset
            if (ang > 360) { ang -= 360; } //correct for offset overshoot.
            return ang;
        }
        return 0;
    }

    /**
     * @return the unbounded steering error, in degrees
     */
    public double getError(double target) {
        return target - getAngle();
    }

    /**
     * @return the steering error bounded to [-180, 180] degrees
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
