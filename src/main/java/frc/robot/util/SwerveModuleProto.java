package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import org.jetbrains.annotations.Nullable;

import static frc.robot.util.MathUtils.boundHalfDegrees;
import static frc.robot.util.MathUtils.round;

@SuppressWarnings("unused")
public class SwerveModuleProto extends SwerveModule {
    //----------------------------------------------------------------------------------------------
    //  All of the following code is documented in the SwerveModule abstract class
    //    You should be able to hover over the methods to view the JavaDocs from there
    //----------------------------------------------------------------------------------------------


    //Module Variables
    private final int id;
    private final double offsetDeg;

    //Drive objects for the Module
    private final CANSparkMax drive;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    //Steering objects for the Module (encoder will be analogInput OR canCoder)
    public final TalonSRX steer;
    @Nullable private final AnalogInput analogInput; //Nullable because it may not be used
    @Nullable public final CANCoder canCoder; //Nullable because it may not be used

    //Steer PID (Manual) Variables
    private final double kP, kI, kD;
    private double sumError = 0, lastError = 0;

    /**
     * @param id ID of the module's motors
     * @param offsetDeg Offset of the module in degrees
     * @param steerPid PID constants for the module steering
     */
    public SwerveModuleProto(int id, int offsetDeg, PID steerPid) {
        //Store PID constants and the module offset
        kP = steerPid.kP; kI = steerPid.kI; kD = steerPid.kD; this.id = id; this.offsetDeg = offsetDeg;

        //Create the SparkMax for the drive motor, and configure the units for its encoder
        drive = new CANSparkMax(id, MotorType.kBrushless); drive.restoreFactoryDefaults();
        driveEncoder = drive.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        //Get the Drive PID Controller and configure it for Velocity PID
        drivePID = drive.getPIDController();
        drivePID.setP(.0001);
        drivePID.setI(0);
        drivePID.setD(.0001);
        drivePID.setIZone(0);
        drivePID.setFF(.000175);
        drivePID.setOutputRange(-1, 1);

        //Create the Steer TalonSRX
        steer = new TalonSRX(id);//set in id ())create talon object

        //Create the encoder based on Constants.ModuleConstants.kSteerEncoderType
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder) {
            //Create the CANCoder and configure it to work as the RemoteSensor0 for the steer motor
            canCoder = new CANCoder(id + 4);
            canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            canCoder.setPositionToAbsolute();
            steer.configRemoteFeedbackFilter(canCoder, 0);
            steer.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
            steer.config_kP(0, kP);
            steer.config_kI(0, kI);
            steer.config_kD(0, kD);
            steer.config_kF(0, steerPid.kF);
            steer.setSelectedSensorPosition(canCoder.getAbsolutePosition());

            analogInput = null;
        }else {
            //Create the AnalogInput for the Steer Encoder
            analogInput = new AnalogInput(id - 1);
            canCoder = null;
        }

        //Reset the encoders
        resetEncoders();
    }

    @Override
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public double getAbsoluteEncoderRad() {
        return Math.toRadians(getAngle());
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        //Ignore small states like when we let go of left stick so wheels don't default to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        //Log some info about the target state
        double target = state.angle.getDegrees();
        SmartDashboard.putString("Swerve[" + id + "] Set", "A: " + round(target, 1) + " S: " + round(state.speedMetersPerSecond, 1));

        //Either run the CanCoder logic for steering, or the AnalogInput logic
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            //Get the error for the angle, assuming + error is clockwise
            double errorAng = boundHalfDegrees(target - getAngle());
            //Error has to be negated since Positive is CCW and Negative is CW
            double pos = steer.getSelectedSensorPosition() + (-errorAng) * (4096D/360D);
            steer.set(TalonSRXControlMode.Position, pos);
        }else {
            double error = getModifiedError(target);
            sumError += error * 0.02;
            double errorChange = (error-lastError)/0.02;
            double pidOutput = error * kP + kI * sumError + kD * errorChange;
            steer.set(ControlMode.PercentOutput, pidOutput);
            lastError = error;
        }

        //Drive Speed with spark and PID (or by percent output using the 2nd line)
        drivePID.setReference(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.neoMaxRPM, CANSparkMax.ControlType.kVelocity);
        //drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    @Override
    public void stop() {
        drive.set(0);
        steer.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    /**
     * Returns current angle (Positive CCW, Negative CW)
     * @return the current angle of the module's wheel [-180, 180]
     */
    public double getAngle() {
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            return -(MathUtils.restrictAngle(steer.getSelectedSensorPosition()*360D/4096D + offsetDeg) - 180);
        }else if (analogInput != null) {
            double ang = analogInput.getValue(); //analog in on the Rio
            ang = ang*360/Constants.potMax + offsetDeg + 90; //Convert to compass type heading + offset
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
}
