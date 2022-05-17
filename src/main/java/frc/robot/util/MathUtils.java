package frc.robot.util;

import edu.wpi.first.wpilibj.drive.Vector2d;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.text.DecimalFormat;

@SuppressWarnings({"SameParameterValue", "unused"})
public class MathUtils {
	/**
	 * Rounds a value to a specified number of place.s
	 * @param value the value to round
	 * @param places the number of places to round to
	 * @return the rounded value
	 */
	public static double round(double value, int places) {
		BigDecimal bd = new BigDecimal(Double.toString(value));
		bd = bd.setScale(places, RoundingMode.HALF_UP);

		return bd.doubleValue();
	}

	/**
	 * Round a value to a specified number of places and return it as a string.
	 * @param value the value to round
	 * @param places the number of places to round to
	 * @return the rounded value as a string
	 */
	public static String roundStr(double value, int places) {
		return (new DecimalFormat("#0." + "0".repeat(Math.max(0, places)))).format(round(value, places));
	}

	/**
	 * I spent like half an hour figuring this out, don't try to figure it out just appreciate the results :)
	 * 0 Degrees is straight forward, 90 degrees is to the right, 180 degrees is backwards, 270 degrees is to the left
	 * Aka clockwise degrees and 0 is straight forward on the joystick :)
	 * @param x the X value of a coordinate
	 * @param y the Y value of a coordinate
	 */
	public static double getHeading(double x, double y) {
		if (x == 0 && y == 0) { return 0; }

		double angle = (360D - ((Math.atan2(y, x)*180D/Math.PI) + 180D)) - 90D;
		if (angle < 0D) {
			angle = 270D + (90D - Math.abs(angle));
		}
		return angle;
	}

	/**
	 * Used to re-obtain the X value of the point on a unit circle from an angle
	 * The angles are in degrees from getHeading()
	 * @param angle The angle (from getHeading()) to get the X value for
	 */
	public static double getHeadingX(double angle) {
		//Ensure values are [0, 360)
		while (angle > 360) { angle -= 360; }
		while (angle < 0) { angle += 360; }

		if (angle >= 0 && angle <= 90) {
			return Math.cos(Math.toRadians(90 - angle));
		}else if (angle >= 90 && angle <= 270) {
			return Math.cos(-Math.toRadians(angle - 90));
		}else if (angle >= 270 && angle <= 360) {
			return -Math.cos(Math.toRadians(270 - angle));
		}
		return 0;
	}

	/**
	 * Used to re-obtain the Y value of the point on a unit circle from an angle
	 * The angles are in degrees from getHeading()
	 * @param angle The angle (from getHeading()) to get the Y value for
	 */
	public static double getHeadingY(double angle) {
		//Ensure values are [0, 360)
		while (angle > 360) { angle -= 360; }
		while (angle < 0) { angle += 360; }

		if (angle >= 0 && angle <= 90) {
			return Math.sin(Math.toRadians(90 - angle));
		}else if (angle >= 90 && angle <= 270) {
			return Math.sin(-Math.toRadians(angle - 90));
		}else if (angle >= 270 && angle <= 360) {
			return -Math.sin(Math.toRadians(270 - angle));
		}
		return 0;
	}

	/**
	 * Takes a value and shifts it towards 0 by a specified amount
	 * @param value the value to shift
	 * @param shift the amount to shift it
	 * @return the shifted value
	 */
	public static double approachZero(double value, double shift) {
		if (value >= 0) {
			return Math.max(0, value - shift);
		}else if (value < 0) {
			return Math.min(0, value + shift);
		}
		return 0;
	}

	/**
	 * Returns a speed value from [-1, 1] based on joystick X and Y inputs
	 * More critically it's snapped to the unit circle so X=1 Y=1 won't be sqrt(2)
	 * @param X the X of a coordinate [-1, 1]
	 * @param Y the Y of a coordinate [-1, 1]
	 */
	public static double getJoystickSpeed(double X, double Y) {
		Vector2d vector = new Vector2d(X, Y);

		double angle = Math.atan2(vector.x, vector.y);
		double maxMagnitude = Math.abs(vector.x) > Math.abs(vector.y)
				? 1 / Math.sin(angle)
				: 1 / Math.cos(angle);
		return Math.abs(vector.magnitude() / maxMagnitude);
	}

	/**
	 * @return the angle given but restricted to [0, 360)
	 * @param angle the angle in degrees to restrict
	 */
	public static double restrictAngle(double angle) {
		while (angle > 360) { angle -= 360; }
		while (angle < 0) { angle += 360; }
		return angle;
	}

	/**
	 * @return the angle bounded to [-180, 180] degrees
	 * @param angle the angle in degrees to bound
	 */
	public static double boundHalfDegrees(double angle) {
		while (angle >= 180.0) angle -= 360.0;
		while (angle < -180.0) angle += 360.0;
		return angle;
	}

	/**
	 * @param a the base
	 * @param b the exponent
	 * @return A to the power of B maintaining the sign of A
	 */
	public static double powAxis(double a, double b) {
		if (a >= 0) {
			return Math.pow(a, b);
		}else {
			return -Math.pow(-a, b);
		}
	}
}
