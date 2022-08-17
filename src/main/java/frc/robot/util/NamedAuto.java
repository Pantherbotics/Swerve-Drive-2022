package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.jetbrains.annotations.Nullable;

/**
 * A class that wraps a Command with a name
 */
@SuppressWarnings("unused")
public class NamedAuto {
    private final String name;
    @Nullable private final Rotation2d startRotation;
    @Nullable private final Pose2d startPose;
    @Nullable private final Command command;

    public NamedAuto(String name, @Nullable Tuple<Command, Rotation2d, Pose2d> data) {
        this.name = name;
        if (data == null) {
            this.command = null; this.startRotation = null; this.startPose = null;
        }else {
            this.command = data.getA();
            this.startRotation = data.getB();
            this.startPose = data.getC();
        }
    }

    public String getName() {
        return name;
    }

    public @Nullable Command getCommand() {
        return command;
    }

    public @Nullable Rotation2d getStartRotation() {
        return startRotation;
    }

    public @Nullable Pose2d getStartPose() {
        return startPose;
    }

}