package frc.robot.util;

public abstract class SwerveModule {
    /**
     * @param velocity is [-1, 1] format for the module to interpret itself
     * @param angle is [0, 360) format for the module to interpret itself
     */
    abstract public void update(double velocity, double angle);

    /**
     * @return true if the module steering is at the desired angle
     */
    abstract public boolean isAtTarget();
}
