package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class helps clarify some inputs and will allow us to easily adapt a SwerveModuleSDS in the future
 */
@SuppressWarnings({"unused", "FieldCanBeLocal"})
public abstract class SwerveModule {

	/**
	 * @return The velocity of the module in m/s
	 */
	public abstract double getDriveVelocity();

	/**
	 * @return the SwerveModuleState of the module from its velocity in m/s and encoder rotation
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
	}

	/**
	 * @param state The new SwerveModuleState, it will NOT be optimized in this method
	 */
	public abstract void setDesiredState(SwerveModuleState state);

	/**
	 * Stops the module by stopping drive and steer motors
	 */
	public abstract void stop();

	/**
	 * Reset the module encoders
	 */
	public abstract void resetEncoders();

	/**
	 * Returns current angle (Positive CCW, Negative CW)
	 * @return the current angle of the module's wheel [-180, 180]
	 */
	public abstract double getAngle();

	/**
	 * Returns current angle in rad (Positive CCW, Negative CW)
	 * @return the current angle of the module's wheel in radians [-pi, pi]
	 */
	public abstract double getAbsoluteEncoderRad();

}
