package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Odometer {
	public double x = 0, y = 0;

	public void update(double x, double y) {
		this.x += x;
		this.y += y;
	}

	public void resetPosition(Pose2d pose2d) {

	}
}
