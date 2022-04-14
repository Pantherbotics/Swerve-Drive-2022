package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunDriveMode;
import frc.robot.subsystems.Drivetrain;

@SuppressWarnings("unused")
public class RobotContainer {
    //Subsystems
    private final Drivetrain drivetrain = new Drivetrain(this);

    //Joysticks
    public final XboxController pJoy = new XboxController(Constants.pJoyID);
    private final JoystickButton joyBA = new JoystickButton(pJoy, 1);
    private final JoystickButton joyBB = new JoystickButton(pJoy, 2);
    private final JoystickButton joyBX = new JoystickButton(pJoy, 3);
    private final JoystickButton joyBY = new JoystickButton(pJoy, 4);

    public RobotContainer() {
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new RunCommand(() ->
                drivetrain.runSwerve(pJoy.getRawAxis(0), pJoy.getRawAxis(1), pJoy.getRawAxis(4), pJoy.getRawAxis(5)), drivetrain
        ));

        joyBA.whenPressed(new RunDriveMode(drivetrain, 1));
        joyBB.whenPressed(new RunDriveMode(drivetrain, 2));
        joyBX.whenPressed(new RunDriveMode(drivetrain, 3));
        joyBY.whenPressed(new RunDriveMode(drivetrain, 4));
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Mode", drivetrain.mode);
        SmartDashboard.putNumber("Gyro", drivetrain.getGyroRot());
        SmartDashboard.putNumber("Gyro Target", drivetrain.targetGyro);
    }

    public double powAxis(double a, double b) {
        if (a >= 0) {
            return Math.pow(a, b);
        }else {
            return -Math.pow(-a, b);
        }
    }

    public double gyroTarget = 0;
    public void init() {
    }

    public void whenDisabled() {}
}
