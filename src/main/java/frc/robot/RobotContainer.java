package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunSwerveJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoPaths;

@SuppressWarnings("unused")
public class RobotContainer {
    //Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    public final SendableChooser<Double> speedChooser;
    public final AutoPaths autoPaths = new AutoPaths(drivetrain);

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
