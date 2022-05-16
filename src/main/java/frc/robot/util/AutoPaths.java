package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;


//-------------------------------------------------------------------------------------------------
//      PathWeaver Notes:
// 1. In order to go backwards, you must keep track of the start and end nodes, and reverse the spline
// 2.
//-------------------------------------------------------------------------------------------------

@SuppressWarnings("unused")
public class AutoPaths {
    private final Drivetrain drivetrain;
    public ArrayList<NamedCommand> paths = new ArrayList<>();

    //Define PID controllers for tracking trajectory
    private final PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    private final PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);


    public AutoPaths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        paths.add(
                new NamedCommand(
                        "Forwards",
                        getAutoCmdFromTrajectories(true, "Forwards")
                )
        );

        paths.add(
                new NamedCommand(
                        "Backwards",
                        getAutoCmdFromTrajectories(true, "Backwards")
                )
        );

        paths.add(
                new NamedCommand(
                        "Left",
                        getAutoCmdFromTrajectories(true, "Left")
                )
        );

        paths.add(
                new NamedCommand(
                        "Right",
                        getAutoCmdFromTrajectories(true, "Right")
                )
        );

        paths.add(
                new NamedCommand(
                        "CircleNoRot",
                        getAutoCmdFromTrajectories(true, "CircleNoRot")
                )
        );

        paths.add(
                new NamedCommand(
                        "CircleRot",
                        getAutoCmdFromTrajectories(true, "CircleRot")
                )
        );

        paths.add(
                new NamedCommand(
                        "Curve",
                        getAutoCmdFromTrajectories(true, "Curve")
                )
        );

        /*
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.DriveConstants.kDriveKinematics);
        paths.add(
                new NamedCommand(
                        "Test",
                        wrapTrajectories(true, (PathPlannerTrajectory) TrajectoryGenerator.generateTrajectory(
                                new Pose2d(2, 2, Rotation2d.fromDegrees(0.0)),
                                List.of(),
                                new Pose2d(4, 2, Rotation2d.fromDegrees(90)),
                                trajectoryConfig
                        ))
                )
        );
        */
    }

    public @Nullable Command getAutoCmdFromTrajectories(boolean firstTraj, String... trajectoryNames) {
        List<PathPlannerTrajectory> trajectories = new ArrayList<>();
        for (String name : trajectoryNames) {
            trajectories.add(loadTrajectory(name));
        }
        return wrapTrajectories(firstTraj, trajectories.toArray(PathPlannerTrajectory[]::new));
    }

    private PathPlannerTrajectory loadTrajectory(String name) {
        return PathPlanner.loadPath(name, Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    private @Nullable Command wrapTrajectories(boolean firstTraj, PathPlannerTrajectory... trajectories) {
        if (trajectories == null || trajectories.length == 0) { return null; }

        List<Command> commands = new ArrayList<>();
        for (PathPlannerTrajectory trajectory : trajectories) {
            // Construct command to follow trajectory
            commands.add(new PPSwerveControllerCommand(
                    trajectory, drivetrain::getPose, Constants.DriveConstants.kDriveKinematics,
                    xController, yController, thetaController,
                    drivetrain::setModuleStatesAuto, drivetrain
            ));
        }

        //Add some init and wrap-up commands (Zero odometry before, and stop modules after)
        if (firstTraj) {
            commands.add(0, new InstantCommand(() -> drivetrain.resetOdometry(trajectories[0].getInitialPose())));
        }
        //commands.add(new InstantCommand(drivetrain::stopModules));
        return new SequentialCommandGroup(commands.toArray(Command[]::new));
    }
}