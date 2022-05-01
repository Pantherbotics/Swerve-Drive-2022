package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

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
    private final double steerOffsetRad;

    /**
     * @param id The id of the motors
     * @param offset The angle offset of the module in radians
     */
    public SwerveModuleProto(int id, double offset) {
        this.id = id;
        this.steerOffsetRad = offset;
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

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    //NEOs return rotations for getPosition, so convert ticks on TalonSRX to rotations, and then apply
    // The conversion we could add for NEOs, but not for Talons
    public double getTurningPosition() {
        return (steer.getSelectedSensorPosition()/2048D) * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    //NEOs return RPM, so convert the ticks per 100 to RPM
    public double getTurningVelocity() {
        return (((steer.getSelectedSensorVelocity()*10)/2048D)*60D) * ModuleConstants.kTurningEncoderRPM2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        double angle = analogInput.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= steerOffsetRad;
        return angle;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        steer.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //If statement ignores when we let go of left stick so wheels aren't defaulting to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        //If we want velocity PID control, this line will work instead:
        //drivePID.setReference(state.speedMetersPerSecond*60D, CANSparkMax.ControlType.kVelocity);
        drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steer.set(ControlMode.PercentOutput, steerPID.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + id + "] state", state.toString());
    }

    public void stop() {
        drive.set(0);
        steer.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
