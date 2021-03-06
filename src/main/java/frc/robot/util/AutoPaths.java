package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import lombok.Getter;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;


//-------------------------------------------------------------------------------------------------
//      PathPlanner Notes:
// 1. When picking up game pieces, remember you can reverse a point so the robot bounces back from it (quicker)
//-------------------------------------------------------------------------------------------------

@SuppressWarnings("unused")
public class AutoPaths {
    private final Drivetrain drivetrain;
    @Getter private final ArrayList<NamedAuto> paths = new ArrayList<>();

    //Define PID controllers for tracking trajectory
    private final PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    private final PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    public AutoPaths(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        thetaController.enableContinuousInput(-Math.PI, Math.PI); //Hmm, indicates the gyro should be [-pi, pi]

        paths.add(
                new NamedAuto(
                        "Forwards",
                        getAutoCmdFromTrajectories(true, "Forwards")
                )
        );

        paths.add(
                new NamedAuto(
                        "Backwards",
                        getAutoCmdFromTrajectories(true, "Backwards")
                )
        );

        paths.add(
                new NamedAuto(
                        "Left",
                        getAutoCmdFromTrajectories(true, "Left")
                )
        );

        paths.add(
                new NamedAuto(
                        "Right",
                        getAutoCmdFromTrajectories(true, "Right")
                )
        );

        paths.add(
                new NamedAuto(
                        "CircleNoRot",
                        getAutoCmdFromTrajectories(true, "CircleNoRot")
                )
        );

        paths.add(
                new NamedAuto(
                        "CircleRot",
                        getAutoCmdFromTrajectories(true, "CircleRot")
                )
        );

        paths.add(
                new NamedAuto(
                        "Curve",
                        getAutoCmdFromTrajectories(true, "Curve")
                )
        );

        paths.add(
                new NamedAuto(
                        "Stress1",
                        getAutoCmdFromTrajectories(true, "Stress1")
                )
        );

        paths.add(
                new NamedAuto(
                        "Stress2",
                        getAutoCmdFromTrajectories(true, "Stress2")
                )
        );

        paths.add(
                new NamedAuto(
                        "Test1",
                        getAutoCmdFromTrajectories(true, "Test1")
                )
        );

        paths.add(
                new NamedAuto(
                        "Test Initial Rotation Calibration",
                        getAutoCmdFromTrajectories(true, "InitRotTest")
                )
        );
    }

    /**
     * @param firstTraj If the trajectory is the first trajectory in the path, and should have the odometry calibrated
     * @param trajectoryNames The names of the trajectories to be loaded
     * @return the Command that will run the controller to follow the trajectories
     */
    public @Nullable Tuple<Command, Rotation2d, Pose2d> getAutoCmdFromTrajectories(boolean firstTraj, String... trajectoryNames) {
        List<PathPlannerTrajectory> trajectories = new ArrayList<>();
        for (String name : trajectoryNames) {
            trajectories.add(loadTrajectory(name));
        }
        return wrapTrajectories(firstTraj, trajectories.toArray(PathPlannerTrajectory[]::new));
    }

    /**
     * @param name The name of the trajectory to load
     * @return The PathPlannerTrajectory loaded from the trajectory file
     */
    private PathPlannerTrajectory loadTrajectory(String name) {
        //Load the PathPlannerTrajectory using PathPlanner's library
        return PathPlanner.loadPath(name, Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    /**
     * @param firstTraj If the trajectory is the first trajectory in the path, and should have the odometry calibrated
     * @param trajectories The trajectories to be wrapped together
     * @return the Command that will run the controller to follow the trajectories
     */
    private @Nullable Tuple<Command, Rotation2d, Pose2d> wrapTrajectories(boolean firstTraj, PathPlannerTrajectory... trajectories) {
        //Add a check in case trajectories are null, or empty (maybe the default is no auto, stuff like that)
        if (trajectories == null || trajectories.length == 0) { return null; }

        //Compile a list of commands of PPSwerveControllerCommand to follow the trajectories
        List<Command> commands = new ArrayList<>();
        for (PathPlannerTrajectory trajectory : trajectories) {
            // Construct command to follow trajectory
            commands.add(new PPSwerveControllerCommand(
                    trajectory, drivetrain::getPose, Constants.DriveConstants.kDriveKinematics,
                    xController, yController, thetaController,
                    drivetrain::setModuleStates, drivetrain
            ));
        }

        //Add the initial command to calibrate the odometry
        if (firstTraj) {
            commands.add(0, new InstantCommand(() -> drivetrain.resetOdometry(trajectories[0].getInitialState().holonomicRotation, trajectories[0].getInitialPose())));
        }
        return Tuple.of(new SequentialCommandGroup(commands.toArray(Command[]::new)), trajectories[0].getInitialState().holonomicRotation, trajectories[0].getInitialPose());
    }
}