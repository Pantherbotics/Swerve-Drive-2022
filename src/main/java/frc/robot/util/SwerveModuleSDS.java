package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class SwerveModuleSDS extends SwerveModule {
    //Motors & Encoders
    private final WPI_TalonFX steer;
    private final WPI_TalonFX drive;
    private final CANCoder encoder;

    //Swerve Module Variables
    public final double angleOffset;
    public final double maxDriveRPM = 5000;
    public final double revsPer100ms = ((maxDriveRPM / 60D) / 10D); //Max Revolutions per 100ms
    public final double velFactor = revsPer100ms * 2048; //Converts [-1,1] to Falcon500 velocity in ticks/100ms

    //Constructor Run once when drive is turned on.
    public SwerveModuleSDS(int id, int steerID, int driveID, int canCoderID, double angleOffset, PID drivePID, PID steerPID) {
        this.angleOffset = angleOffset;
        steer = PID.setupFalcon(steerID, false, steerPID, 0, 30);
        drive = PID.setupFalcon(driveID, false, drivePID, 0, 30);
        encoder = new CANCoder(canCoderID);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); //Sets range to [0, 360)

        //Run Every 100ms to update the Dashboard
        new Notifier(() -> SmartDashboard.putNumber("SwerveModule" + id + " Angle", getAngle())).startPeriodic(100D/1000D);
    }

    private double steerTarget = 0;
    /**
        * Setter Run each time wheel is updated.
        * @param velocity: target speed of the wheel in [-1, 1]
        * @param angle: target angle of the wheel in degrees (handles any angle, >360, <0)
        */
    //This may require edge case handling but for now I believe it will work (given inputs are in range)
    public void updateModule(double velocity, double angle) {
        //Normalize the angle to [0, 360)
        while (angle > 360) { angle -= 360; }
        while (angle < 0) { angle += 360; }

        double currAngle = getAngle();
        double error = getShortestError(angle, currAngle);
        double deltaSteerPos = error * Constants.angleGearRatio * 2048;
        steerTarget = steer.getSelectedSensorPosition() + deltaSteerPos;
        steer.set(ControlMode.Position, steerTarget);
        drive.set(ControlMode.Velocity, velocity * velFactor);
    }

    private double getAngle() {
        return encoder.getAbsolutePosition() + angleOffset; //Conveniently returns angle in degrees [0, 360)
    }

    private double getShortestError(double target, double current) {
        double error = target - current;
        if (error > 180) { error -= 360; }
        else if (error < -180) { error += 360; }
        return error;
    }

    @Override
    public void update(double velocity, double angle) {
        updateModule(velocity, angle);
    }

    @Override
    public boolean isAtTarget() {
        return Math.abs(steer.getSelectedSensorPosition() - steerTarget) <= 128;
    }
}
