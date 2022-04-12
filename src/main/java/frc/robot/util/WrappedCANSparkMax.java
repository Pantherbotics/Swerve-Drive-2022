package frc.robot.util;

import com.revrobotics.CANSparkMax;

@SuppressWarnings("unused")
public class WrappedCANSparkMax extends CANSparkMax {
    protected double mLastSet = Double.NaN;

    public WrappedCANSparkMax(int deviceNumber, MotorType motorType) {
        super(deviceNumber, motorType);
    }

    public double getLastSet() {
        return mLastSet;
    }


    //credit to 1323 and 254 for this set function
    //helps minimize CAN overhead by ignoring redundant commands.

    @Override
    public void set(double value) {
        if (value != mLastSet) {
            mLastSet = value;
            super.set(value);
        }
    }

    //set everything to zero
    public void reset() {}

}