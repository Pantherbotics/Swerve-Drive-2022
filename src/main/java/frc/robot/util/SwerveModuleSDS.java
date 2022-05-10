package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class SwerveModuleSDS {
    //Motors & Encoders
    private final WPI_TalonFX steer;
    private final WPI_TalonFX drive;
    private final CANCoder canCoder;

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
        canCoder = new CANCoder(canCoderID);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); //Sets range to [0, 360)
        steer.configRemoteFeedbackFilter(canCoder, 0);
        steer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 20);
        steer.config_kP(0, 0.2);
        steer.config_kI(0, 0.0);
        steer.config_kD(0, 0.0);
        steer.config_kF(0, 0.0);


        //Run Every 100ms to update the Dashboard
        new Notifier(() -> SmartDashboard.putNumber("SwerveModule" + id + " Angle", getAngle())).startPeriodic(100D/1000D);


        //CRITICAL: We can either calculate and use the position PID inside the Falcon500, or use
        // data from the CANCoder, supplied to the steer motor TalonFX.
        //steer.configRemoteFeedbackFilter(new CANCoder(0), 0);
        //steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);
    }

    /**
     * Setter Run each time wheel is updated.
     * @param velocity: target speed of the wheel in [-1, 1]
     * @param angle: target angle of the wheel in degrees [0, 360) (can handle any angle, >360, <0)
     */
    //This may require edge case handling but for now I believe it will work (given inputs are in range)
    public void updateModule(double velocity, double angle) {
        //Normalize the angle to [0, 360)
        while (angle > 360) { angle -= 360; }
        while (angle < 0) { angle += 360; }

        steer.set(ControlMode.Position, angle + angleOffset);
        drive.set(ControlMode.Velocity, velocity * velFactor);
    }

    private double getAngle() {
        //Conveniently returns angle in degrees [0, 360)
        return canCoder.getAbsolutePosition() + angleOffset;
    }

    private double getShortestError(double target, double current) {
        //target and current are [0, 360) so one addition/subtraction is sufficient
        double error = target - current;
        if (error > 180) { error -= 360; }
        else if (error < -180) { error += 360; }
        return error;
    }
}