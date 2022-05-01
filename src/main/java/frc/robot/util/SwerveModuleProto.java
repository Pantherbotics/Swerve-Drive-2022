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


@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class SwerveModuleProto extends SwerveModule{
    private final int id;
    private final CANSparkMax drive;
    private final TalonSRX steer;

    private final RelativeEncoder driveEncoder;

    private final SparkMaxPIDController drivePID;
    private final PIDController steerPID;
    private final AnalogInput analogInput;
    private final double steerOffsetRad; //In Radians

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

        steerPID = new PIDController(ModuleConstants.kPTurning, 0, 0);
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


    //TODO convert units
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    //TODO convert units
    public double getTurningPosition() {
        return steer.getSelectedSensorPosition();
    }

    //TODO convert units
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    //TODO convert units
    public double getTurningVelocity() {
        return steer.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = analogInput.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= steerOffsetRad;
        return angle;
    }

    //TODO convert units
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        steer.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    //TODO check that units are correct
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    //TODO verify units
    public void setDesiredState(SwerveModuleState state) {
        //If statement ignores when we let go of left stick so wheels aren't defaulting to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        //drive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steer.set(ControlMode.PercentOutput, steerPID.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + id + "] state", state.toString());
    }

    //TODO make sure that set 0 for a TalonSRX does what we want it to do
    public void stop() {
        drive.set(0);
        steer.set(TalonSRXControlMode.PercentOutput, 0);
    }

    //TODO re-evaluate the SwerveModule class methods and arguments
    @Override
    public void update(double velocity, double angle) {
        setDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
    }

    @Override
    public boolean isAtTarget() {
        return false;
    }
}
