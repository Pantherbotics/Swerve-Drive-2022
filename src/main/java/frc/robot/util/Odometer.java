package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Keeps track of the robot position relative to the field
 */
public class Odometer {
    public double x = 0, y = 0;

    /**
     * @param x The delta X to add to the current value
     * @param y The delta Y to add to the current value
     */
    public void update(double x, double y) {
        this.x += x;
        this.y += y;
    }

    /**
     * @return The current coordinates of the robot
     */
    public Translation2d getPoseMeters() {
        return new Translation2d(x, y);
    }

    /**
     * Reset the odometry to a position
     * @param pose2d The Pose2d containing the new X and Y coordinates
     */
    public void resetPosition(Pose2d pose2d) {
        x = pose2d.getX();
        y = pose2d.getY();
    }
}