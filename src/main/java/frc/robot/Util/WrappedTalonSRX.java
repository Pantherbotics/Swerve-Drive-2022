package frc.robot.Util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class WrappedTalonSRX extends TalonSRX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public WrappedTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
        return mLastSet;
    }


    //credit to 1323 and 254 for this set function
    //helps minimize CAN overhead by ignoring redundant commands.

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }

    //set everything to zero
    public void reset(){
        super.configOpenloopRamp(0, 10);
        super.configClosedloopRamp(0, 10);
        super.configPeakOutputForward(1, 10);
        super.configPeakOutputReverse(-1, 10);
        super.configNominalOutputForward(0, 10);
        super.configNominalOutputReverse(0, 10);
        super.configNeutralDeadband(0.04,10);
        super.configVoltageCompSaturation(0, 10);
        super.configVoltageMeasurementFilter(32, 10);
        super.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        super.configSelectedFeedbackCoefficient(1.0, 0, 10);
        super.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, 10);
        super.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, 10);
        super.configForwardSoftLimitThreshold(0, 10);
        super.configReverseSoftLimitThreshold(0, 10);
        super.configForwardSoftLimitEnable(false, 10);
        super.configReverseSoftLimitEnable(false, 10);
        super.config_kP(0, 0, 10);
        super.config_kI(0, 0, 10);
        super.config_kD(0, 0, 10);
        super.config_kF(0, 0, 10);
        super.config_IntegralZone(0, 0, 10);
        super.configAllowableClosedloopError(0, 0 ,10);
        super.configMaxIntegralAccumulator(0, 0, 10);
        super.configClosedLoopPeakOutput(0, 1.0, 10);
        super.configClosedLoopPeriod(0, 1, 10);
        super.configAuxPIDPolarity(false, 10);
        super.configMotionCruiseVelocity(0, 10);
        super.configMotionAcceleration(0, 10);
        super.configMotionProfileTrajectoryPeriod(0, 10);
        super.configSetCustomParam(0, 0, 10);
        super.configPeakCurrentLimit(0, 10);
        super.configContinuousCurrentLimit(0, 10);
        super.enableCurrentLimit(false);
      }

}