package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
public class AutoPaths {
    private final Drivetrain drivetrain;
    public ArrayList<NamedCommand> paths = new ArrayList<>();

    // 1. Create trajectory settings
    private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.DriveConstants.kDriveKinematics);
    // 3. Define PID controllers for tracking trajectory
    private final PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    private final PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);


    public AutoPaths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        paths.add(
                new NamedCommand(
                        "Test1",
                        getAutoCmdFromTrajectories("Test1")
                )
        );
    }

    public @Nullable Command getAutoCmdFromTrajectories(String... trajectoryNames) {
        List<Trajectory> trajectories = new ArrayList<>();
        for (String name : trajectoryNames) {
            trajectories.add(loadTrajectory(name));
        }
        return wrapTrajectories(trajectories.toArray(Trajectory[]::new));
    }

    private Trajectory loadTrajectory(String name) {
        String trajectoryJSON = "paths/" + name + ".wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, false);
        }
        return null;
    }

    private @Nullable Command wrapTrajectories(Trajectory... trajectories) {
        if (trajectories == null || trajectories.length == 0) { return null; }

        List<Command> commands = new ArrayList<>();
        for (Trajectory trajectory : trajectories) {
            // Construct command to follow trajectory
            commands.add(new SwerveControllerCommand(
                    trajectory, drivetrain::getPose, Constants.DriveConstants.kDriveKinematics,
                    xController, yController, thetaController,
                    drivetrain::setModuleStates, drivetrain
            ));
        }

        //Add some init and wrap-up commands (Zero odometry before, and stop modules after)
        commands.add(0, new InstantCommand(() -> drivetrain.resetOdometry(trajectories[0].getInitialPose())));
        commands.add(new InstantCommand(drivetrain::stopModules));
        return new SequentialCommandGroup(commands.toArray(Command[]::new));
    }
}