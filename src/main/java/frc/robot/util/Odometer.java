package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Odometer {
    public double x = 0, y = 0;

    public void update(double x, double y) {
        this.x += x;
        this.y += y;
    }

    public Translation2d getPoseMeters() {
        return new Translation2d(x, y);
    }

    public void resetPosition(Pose2d pose2d) {
        x = pose2d.getX();
        y = pose2d.getY();
    }
}