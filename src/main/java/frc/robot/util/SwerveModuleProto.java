package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

//Units of everything were checked and verified as of May 1st, 2022
@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class SwerveModuleProto {
    private final int id;
    private final CANSparkMax drive;
    private final TalonSRX steer;

    private final RelativeEncoder driveEncoder;

    private final SparkMaxPIDController drivePID;
    private final PIDController steerPID;
    private final AnalogInput analogInput;
    private final double offset; //In Degrees

    private final double potMax = 3798;

    /**
     * @param id The id of the motors
     * @param offset The angle offset of the module in radians
     */
    public SwerveModuleProto(int id, double offset) {
        this.id = id;
        this.offset = offset;
        analogInput = new AnalogInput(id - 1);

        drive = new CANSparkMax(id, MotorType.kBrushless); drive.restoreFactoryDefaults();
        steer = new TalonSRX(id);

        driveEncoder = drive.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        steerPID = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

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
        return (steer.getSelectedSensorPosition()/2048D) * ModuleConstants.kTurningEncoderRot2Rad;
        //return getAbsoluteEncoderRad() + Math.PI; //Probably this one
        //return getAbsoluteEncoderRad();
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
        //TODO i suspect that drive velocity is working, but getTurningPosition is not
        //The TalonSRX does not have an encoder to drive its pid, it's using the analog input
        //Probably just have to get wheel angle instead
        SmartDashboard.putNumber("Swerve[" + id + "] Drive Vel (RPM)", getDriveVelocity());
        SmartDashboard.putNumber("Swerve[" + id + "] Wheel Rot (Rad)", getTurningPosition());
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //If statement ignores when we let go of left stick so wheels aren't defaulting to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        if (Constants.optimizeModuleStates) { state = SwerveModuleState.optimize(state, getState().angle); }

        //If we want velocity PID control, this line will work instead:
        //drivePID.setReference(state.speedMetersPerSecond*60D, CANSparkMax.ControlType.kVelocity);
        drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steer.set(ControlMode.PercentOutput, steerPID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public void stop() {
        drive.set(0);
        steer.set(ControlMode.PercentOutput, 0);
    }

    //Returns angle in radians [-pi, pi]
    public double getAbsoluteEncoderRad() {
        double angle = analogInput.getValue() / potMax;
        angle *= (2.0 * Math.PI);
        angle -= Math.PI; //Convert from [0, 2pi] to [-pi, pi]
        angle -= Math.toRadians(offset);
        while (angle > Math.PI) { angle -= 2 * Math.PI; }
        while (angle < -Math.PI) { angle += 2 * Math.PI; }
        return angle;
    }
}
