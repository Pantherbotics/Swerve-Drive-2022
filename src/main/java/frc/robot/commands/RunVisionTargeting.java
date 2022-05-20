package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

@SuppressWarnings("unused")
public class RunVisionTargeting extends CommandBase {
	private final Drivetrain drivetrain;
	private final Limelight limelight;
	public RunVisionTargeting(Drivetrain drivetrain, Limelight limelight) {
		this.drivetrain = drivetrain;
		this.limelight = limelight;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (limelight.hasTarget()) {
			drivetrain.setLimelightYaw(limelight.getTarget().getYaw());
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.setLimelightYaw(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
