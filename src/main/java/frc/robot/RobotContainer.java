package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.subsystems.Drivetrain;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@SuppressWarnings("unused")
public class RobotContainer {
    //Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    public final SendableChooser<Double> speedChooser;
    public final Map<String, Trajectory> trajectories = new HashMap<>();

    //Joysticks
    private final XboxController pJoy = new XboxController(Constants.pJoyID);
    private final JoystickButton joyBA = new JoystickButton(pJoy, 1); //Button A
    private final JoystickButton joyBB = new JoystickButton(pJoy, 2); //Button B
    private final JoystickButton joyBX = new JoystickButton(pJoy, 3); //Button X
    private final JoystickButton joyBY = new JoystickButton(pJoy, 4); //Button Y
    private final POVButton sJoyPOVN = new POVButton(pJoy, 0);   //POV North
    private final POVButton sJoyPOVS = new POVButton(pJoy, 180); //POV South
    private final POVButton sJoyPOVW = new POVButton(pJoy, 270); //POV West
    private final POVButton sJoyPOVE = new POVButton(pJoy, 90);  //POV East

    public RobotContainer(Robot robot) {
        speedChooser = robot.speedChooser;
        double exp = 7D/3D;
        drivetrain.setDefaultCommand(new RunSwerveJoystick(
                drivetrain,
                () -> -powAxis(pJoy.getRawAxis(OIConstants.kDriverYAxis), exp)/speedChooser.getSelected(),
                () -> powAxis(pJoy.getRawAxis(OIConstants.kDriverXAxis), exp)/speedChooser.getSelected(),
                () -> pJoy.getRawAxis(OIConstants.kDriverRotAxis)/speedChooser.getSelected()));

        //For easy calibration, use this code instead to have all wheels drive forward
        //drivetrain.setDefaultCommand(new RunSwerveJoystick(drivetrain, () -> 0.1, () -> 0.0, () -> 0.0));

        configureButtonBindings();

        //WPI warns that these take some time to load, so load them immediately before auto starts
        loadTrajectories();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(pJoy, 2).whenPressed(drivetrain::zeroHeading);
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)
                ),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

        Trajectory trajectory = trajectories.get("First Ball");

        //Trajectory numbers/coordinates are in meters from origin/start
        /*
        trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0)),
                new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);
        */

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory, drivetrain::getPose, DriveConstants.kDriveKinematics,
                xController, yController, thetaController,
                drivetrain::setModuleStates, drivetrain
        );

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(drivetrain::stopModules)
        );
    }

    public void loadTrajectories() {
        File[] jsonPaths = new File(String.valueOf(Filesystem.getDeployDirectory().toPath())).listFiles();
        if (jsonPaths == null) { DriverStation.reportError("Could not automatically load trajectory path JSONs", false); return; }
        for (File file : jsonPaths) {
            String name = file.getName().substring(0, file.getName().length() - 12); //Crop .wpilib.json
            loadTrajectory(name);
        }
        DriverStation.reportWarning("Trajectories Loaded", false);
    }

    public void loadTrajectory(String name) {
        String trajectoryJSON = "paths/" + name + ".wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectories.put(name, TrajectoryUtil.fromPathweaverJson(trajectoryPath));
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, false);
        }
    }

    public double powAxis(double a, double b) {
        if (a >= 0) {
            return Math.pow(a, b);
        }else {
            return -Math.pow(-a, b);
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Gyro", drivetrain.getHeading());
    }
}
