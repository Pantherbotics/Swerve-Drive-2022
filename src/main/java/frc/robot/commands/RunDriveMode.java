package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunDriveMode extends CommandBase {
    private final Drivetrain drivetrain;
    private final int mode;
    public RunDriveMode(Drivetrain drivetrain, int mode) {
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
