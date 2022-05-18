package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.jetbrains.annotations.Nullable;

/**
 * A class that wraps a Command with a name
 */
@SuppressWarnings("unused")
public class NamedAuto {
    private final String name;
    @Nullable private final Pose2d startPose;
    @Nullable private final Command command;

    public NamedAuto(String name, @Nullable Command command) {
        this.name = name;
        this.command = command;
        this.startPose = null;
    }

    public NamedAuto(String name, @Nullable Pair<Command, Pose2d> data) {
        this.name = name;
        if (data == null) {
            this.command = null; this.startPose = null;
        }else {
            this.command = data.getFirst();
            this.startPose = data.getSecond();
        }
    }

    public String getName() {
        return name;
    }

    public @Nullable Command getCommand() {
        return command;
    }

    public @Nullable Pose2d getStartPose() {
        return startPose;
    }
}