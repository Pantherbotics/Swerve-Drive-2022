package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.RunDriveMode;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoPaths;
import frc.robot.util.DriveMode;

import static frc.robot.util.MathUtils.round;

@SuppressWarnings("unused")
public class RobotContainer {
    //Subsystems
    public final Drivetrain drivetrain = new Drivetrain();
    public final SendableChooser<Double> speedChooser;
    public final AutoPaths autoPaths = new AutoPaths(drivetrain);

    //Joysticks and Buttons/Inputs
    private final Joystick pJoy = new Joystick(Constants.OIConstants.kDriverJoyID);
    private final JoystickButton joyBA = new JoystickButton(pJoy, 1); //Button A
    private final JoystickButton joyBB = new JoystickButton(pJoy, 2); //Button B
    private final JoystickButton joyBX = new JoystickButton(pJoy, 3); //Button X
    private final JoystickButton joyBY = new JoystickButton(pJoy, 4); //Button Y
    private final POVButton sJoyPOVN = new POVButton(pJoy, 0);   //POV North
    private final POVButton sJoyPOVNE = new POVButton(pJoy, 45); //POV North East
    private final POVButton sJoyPOVS = new POVButton(pJoy, 180); //POV South
    private final POVButton sJoyPOVSE = new POVButton(pJoy, 135); //POV North East
    private final POVButton sJoyPOVW = new POVButton(pJoy, 270); //POV West
    private final POVButton sJoyPOVE = new POVButton(pJoy, 90);  //POV East

    public RobotContainer(Robot robot) {
        speedChooser = robot.speedChooser;

        drivetrain.setDefaultCommand(new RunSwerveJoystick(
                drivetrain, pJoy,
                speedChooser::getSelected,
                drivetrain::getMode
        ));
        //For easy calibration, use this code instead to have all wheels drive forward
        //drivetrain.setDefaultCommand(new RunSwerveJoystick(drivetrain, () -> 0.1, () -> 0.0, () -> 0.0));

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        joyBB.whenPressed(drivetrain::zeroHeading);

        //Configure multiple swerve modes
        sJoyPOVN.whenPressed(new RunDriveMode(drivetrain, DriveMode.CAR));
        sJoyPOVS.whenPressed(new RunDriveMode(drivetrain, DriveMode.BOAT));
        sJoyPOVE.whenPressed(new RunDriveMode(drivetrain, DriveMode.FO_SWERVE));
        sJoyPOVW.whenPressed(new RunDriveMode(drivetrain, DriveMode.SWERVE));

        sJoyPOVNE.whenPressed(new RunDriveMode(drivetrain, DriveMode.WESTCOAST));
        sJoyPOVSE.whileHeld(new RunDriveMode(drivetrain, DriveMode.TANK));
    }


    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Gyro", drivetrain.getHeading());
        SmartDashboard.putString("Mode", drivetrain.getMode().getName());

        SmartDashboard.putString("Swerve[1] Data", "A: " + round(drivetrain.leftFront.getAngle(),2) + " S: " + round(drivetrain.leftFront.getDriveVelocity(), 2));
        SmartDashboard.putString("Swerve[2] Data", "A: " + round(drivetrain.rightFront.getAngle(),2) + " S: " + round(drivetrain.rightFront.getDriveVelocity(), 2));
        SmartDashboard.putString("Swerve[3] Data", "A: " + round(drivetrain.rightBack.getAngle(),2) + " S: " + round(drivetrain.rightBack.getDriveVelocity(), 2));
        SmartDashboard.putString("Swerve[4] Data", "A: " + round(drivetrain.leftBack.getAngle(),2) + " S: " + round(drivetrain.leftBack.getDriveVelocity(), 2));

        SmartDashboard.putNumber("Robot Heading", drivetrain.getHeading());

        double x = round(drivetrain.getPose().getTranslation().getX(),3);
        double y = round(drivetrain.getPose().getTranslation().getY(),3);
        SmartDashboard.putString("Robot Location", "X: " + x + " Y: " + y);

        //These methods have a warning that they are noisy and may not be useful
        double vx = drivetrain.gyro.getVelocityX();
        double vy = drivetrain.gyro.getVelocityY();
        SmartDashboard.putNumber("Robot Vel", Math.sqrt(vx*vx + vy*vy));
    }
}
