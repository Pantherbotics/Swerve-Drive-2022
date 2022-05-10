package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import org.jetbrains.annotations.Nullable;

//Units of everything were checked and verified as of May 1st, 2022
@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class SwerveModuleProto {
    private final int id;
    private final CANSparkMax drive;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    private final TalonSRX steer;
    @Nullable private final PIDController steerPID;
    @Nullable private final AnalogInput analogInput; //Encoder for Potentiometers
    @Nullable private final CANCoder canCoder; //CanCoder

    private final double offsetDeg;

    /**
     * @param id The id of the motors
     * @param offsetDeg The angle offset of the module in degrees
     */
    public SwerveModuleProto(int id, double offsetDeg) {
        this.id = id;
        this.offsetDeg = offsetDeg;

        steer = new TalonSRX(id);

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

            steerPID = null;
            analogInput = null;
        }else {
            analogInput = new AnalogInput(id - 1);
            steerPID = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
            steerPID.enableContinuousInput(-Math.PI, Math.PI);
            canCoder = null;
        }

        drive = new CANSparkMax(id, MotorType.kBrushless); drive.restoreFactoryDefaults();
        driveEncoder = drive.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        drivePID = drive.getPIDController();
        drivePID.setP(.0001); //works well for swerve
        drivePID.setI(0);
        drivePID.setD(.0001);
        drivePID.setIZone(0);
        drivePID.setFF(.000175); //good for bench, may be increased on the floor
        drivePID.setOutputRange(-1, 1); //min and max to motor

        resetEncoders();
    }

    //public double getDrivePosition() { return driveEncoder.getPosition(); }

    //NEOs return rotations for getPosition, so convert ticks on TalonSRX to rotations, and then apply
    // The conversion we could add for NEOs, but not for Talons
    private double getTurningPosition() {
        //return (steer.getSelectedSensorPosition()/2048D) * ModuleConstants.kTurningEncoderRot2Rad;
        return -getAbsoluteEncoderRad();
    }

    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    //BROKEN (since steer doesn't have an encoder)       NEOs return RPM, so convert the ticks per 100 to RPM
    //public double getTurningVelocity() { return (((steer.getSelectedSensorVelocity()*10)/2048D)*60D) * ModuleConstants.kTurningEncoderRPM2RadPerSec; }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //steer.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        //The TalonSRX does not have an encoder to drive its pid, it's using the analog input
        //Probably just have to get wheel angle instead
        //SmartDashboard.putNumber("Swerve[" + id + "] Drive Vel (RPM)", getDriveVelocity());
        //SmartDashboard.putNumber("Swerve[" + id + "] Wheel Rot (Rad)", getTurningPosition());
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //If statement ignores when we let go of left stick so wheels aren't defaulting to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderRad()));

        SmartDashboard.putNumber("Swerve[" + id + "] SA", state.angle.getRadians());

        //If we want velocity PID control, this line will work instead:
        //drivePID.setReference(state.speedMetersPerSecond*60D, CANSparkMax.ControlType.kVelocity);
        drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            canCoder.setPosition(Math.toDegrees(state.angle.getRadians()) + offsetDeg);
        }else if (steerPID != null) {
            steer.set(ControlMode.PercentOutput, steerPID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        }
    }

    public void setDesiredStateAuto(SwerveModuleState state) {
        state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(-state.angle.getRadians()));
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderRad()));

        SmartDashboard.putNumber("Swerve[" + id + "] SA", state.angle.getRadians());

        //If we want velocity PID control, this line will work instead:
        //drivePID.setReference(state.speedMetersPerSecond*60D, CANSparkMax.ControlType.kVelocity);
        drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            canCoder.setPosition(Math.toDegrees(state.angle.getRadians()));
        }else if (steerPID != null) {
            steer.set(ControlMode.PercentOutput, steerPID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        }
    }

    public void stop() {
        drive.set(0);
        steer.set(ControlMode.PercentOutput, 0);
    }

    //Returns angle in nobody knows
    public double getAbsoluteEncoderRad() {
        if (Constants.kEncoderType == Constants.EncoderType.CanCoder && canCoder != null) {
            return Math.toRadians(canCoder.getAbsolutePosition());
        }else if (analogInput != null) { //It's a Potentiometer
            double angle = analogInput.getValue() / Constants.potMax;
            angle *= (2.0 * Math.PI);
            angle -= Math.PI; //Convert from [0, 2pi] to [-pi, pi]
            angle -= Math.toRadians(offsetDeg);
            while (angle > Math.PI) { angle -= 2 * Math.PI; }
            while (angle < -Math.PI) { angle += 2 * Math.PI; }
            return angle;
        }
        return 0;
    }
}
