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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.*;
import org.jetbrains.annotations.Nullable;

@SuppressWarnings("unused")
public class SwerveModuleProto {
    private final int id;

    //Drive stuff
    private final CANSparkMax drive;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    //Steering stuff
    private final TalonSRX steer;
    @Nullable private final AnalogInput analogInput;
    @Nullable private final CANCoder canCoder;

    //Variable declarations
    double kP, kI, kD, kF;
    double potMax = 3798;
    double sumError = 0, errorChange = 0, lastError = 0;
    double offsetDeg;

    /**
     * @param id ID of the module's motors
     * @param offsetDeg Offset degrees of the module
     * @param pid PID constants for the module steering
     */
    public SwerveModuleProto(int id, int offsetDeg, PID pid) {
        kP = pid.kP; kI = pid.kI; kD = pid.kD; kF = pid.kF; this.id = id;

        this.offsetDeg = offsetDeg;

        //Steering Motor setup
        steer = new TalonSRX(id);//set in id ())create talon object

        if (Constants.kEncoderType == Constants.EncoderType.CanCoder) {
            canCoder = new CANCoder(id + 4);
            canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            canCoder.setPositionToAbsolute();
            steer.configRemoteFeedbackFilter(canCoder, 0);
            steer.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.RemoteSensor0, 0, 20);
            steer.config_kP(0, kP);
            steer.config_kI(0, kI);
            steer.config_kD(0, kD);
            steer.config_kF(0, kF);
            steer.setSelectedSensorPosition(canCoder.getAbsolutePosition());

            analogInput = null;
        }else {
            analogInput = new AnalogInput(id - 1);
            canCoder = null;
        }


        drive = new CANSparkMax(id, MotorType.kBrushless); drive.restoreFactoryDefaults();
        driveEncoder = drive.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        drivePID = drive.getPIDController();
        // set Drive motor PID coefficients
        drivePID.setP(.0001);
        drivePID.setI(0);
        drivePID.setD(.0001);
        drivePID.setIZone(0);
        drivePID.setFF(.000175);
        drivePID.setOutputRange(-1, 1);

        resetEncoders();


        // This class is what I've been searching for...
        // RobotController.getBatteryVoltage();
        // RobotController.getVoltage5V(); //Helpful to interpret analog inputs when source is not 5V
    }

    private double getTurningPosition() {
        return getAbsoluteEncoderRad();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //steer.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        //The TalonSRX does not have an encoder to drive its pid, it's using the analog input
        //Probably just have to get wheel angle instead
        //SmartDashboard.putNumber("Swerve[" + id + "] Drive Vel (RPM)", getDriveVelocity());
        //SmartDashboard.putNumber("Swerve[" + id + "] Wheel Rot (Rad)", getTurningPosition());
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }


    /**
     * @param state The new SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {
        //If statement ignores when we let go of left stick so wheels aren't defaulting to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderRad()));

        SmartDashboard.putNumber("Swerve[" + id + "] SA", state.angle.getRadians());

        //TODO check range of values from here, currently assuming [0, 360), could be [-180, 180]
        double target = Math.toDegrees(state.angle.getRadians());

        //Custom PID loop
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            //target = MathUtils.restrictAngle(target + offsetDeg);
            double errorAng = boundHalfDegrees(target - getAngle());
            double pos = steer.getSelectedSensorPosition() + errorAng * (4096D/360D);

            steer.set(TalonSRXControlMode.Position, pos);
            SmartDashboard.putString("[" + id + "] Data", MathUtils.round(target, 3) + ":" + MathUtils.round(getAngle(), 3));
        }else {
            double error = getModifiedError(target);
            sumError += error * 0.02;
            errorChange = (error-lastError)/0.02;
            double pidOutput = error * kP + kI * sumError + kD * errorChange;
            steer.set(ControlMode.PercentOutput, pidOutput);
            lastError = error;
        }

        //Drive Speed with spark
        drivePID.setReference(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * Constants.neoMaxRPM, CANSparkMax.ControlType.kVelocity);
        drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public void setDesiredStateAuto(SwerveModuleState state) {
        state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(-state.angle.getRadians()));
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderRad()));

        setDesiredState(state);
    }

    public void stop() {
        drive.set(0);
        steer.set(ControlMode.PercentOutput, 0);
    }

    /**
     * @return the current angle of the module's wheel in radians [-pi, pi])
     */
    public double getAbsoluteEncoderRad() {
        return Math.toRadians(getAngle()) - 180;
    }

    /**
     * @return the current angle of the module's wheel [0, 360)
     */
    public double getAngle() {
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            return MathUtils.restrictAngle(steer.getSelectedSensorPosition()*360D/4096D + offsetDeg);
        }else if (analogInput != null) {
            double ang = analogInput.getValue(); //analog in on the Rio
            ang = ang*360/potMax + offsetDeg + 90; //Convert to compass type heading + offset
            if (ang > 360) { ang -= 360; } //correct for offset overshoot.
            return ang;
        }
        return 0;
    }

    public double getPosition() {
        return steer.getSelectedSensorPosition();
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
}
