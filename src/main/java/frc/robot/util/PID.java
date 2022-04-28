package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class PID {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final double allowedError;
    public final double maxPower;
    public PID(double kP, double kI, double kD, double kF, double allowedError, double maxPower) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.allowedError = allowedError;
        this.maxPower = maxPower;
    }

    //Condenses code by calling common neutralMode and sensor methods in fewer lines
    public static WPI_TalonFX setupFalcon(int id, boolean invert, PID pid, int PID_LOOP_IDX, int TIMEOUT_MS) {
        WPI_TalonFX talon = new WPI_TalonFX(id);

        //Configure PID
        talon.configFactoryDefault();
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_LOOP_IDX, TIMEOUT_MS);

        talon.configNominalOutputForward(0, TIMEOUT_MS);
        talon.configNominalOutputReverse(0, TIMEOUT_MS);
        talon.configPeakOutputForward(pid.maxPower,  TIMEOUT_MS);
        talon.configPeakOutputReverse(-pid.maxPower, TIMEOUT_MS);

        talon.configAllowableClosedloopError(PID_LOOP_IDX, pid.allowedError, TIMEOUT_MS);

        talon.config_kP(PID_LOOP_IDX, pid.kP, TIMEOUT_MS);
        talon.config_kI(PID_LOOP_IDX, pid.kI, TIMEOUT_MS);
        talon.config_kD(PID_LOOP_IDX, pid.kD, TIMEOUT_MS);
        talon.config_kF(PID_LOOP_IDX, pid.kF, TIMEOUT_MS);

        //Configure Motor
        talon.setNeutralMode(NeutralMode.Coast);
        talon.setSelectedSensorPosition(0);
        talon.setInverted(invert);
        return talon;
    }
}
