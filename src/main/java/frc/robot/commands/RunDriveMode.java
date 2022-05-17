package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DriveMode;

public class RunDriveMode extends CommandBase {
    private final Drivetrain drivetrain;
    private final DriveMode mode;
    public RunDriveMode(Drivetrain drivetrain, DriveMode mode) {
        this.drivetrain = drivetrain;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        drivetrain.setMode(mode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
