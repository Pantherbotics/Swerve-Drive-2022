/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*This program modified by CWS 8/4/2021,   This will track the motion of an analog
 sensor which rotates beyond 360 deg.
 Note the pot is connected directly to the Robo Rio analog in
    The Talon sensor collection can measure the value without roll up, but it is too slow at 150ms.
    The talon PID cannot accommodate the 0/360 discontinuity
    Spark controls the drive motor (Neo) in speed mode.
 */

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.NamedAuto;

public class Robot extends TimedRobot {
    public final SendableChooser<NamedAuto> autoChooser = new SendableChooser<>();
    public final SendableChooser<Double> speedChooser = new SendableChooser<>();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        speedChooser.setDefaultOption("Normal (Fast 100%)", 1.0);
        speedChooser.addOption("Mild (Medium 50%)", 0.5);
        speedChooser.addOption("Safe (Slow 25%)", 0.25);
        SmartDashboard.putData(speedChooser);

        robotContainer = new RobotContainer(this);

        autoChooser.setDefaultOption("None", new NamedAuto("None", (Command) null));
        for (NamedAuto command : robotContainer.getAutoPaths().getPaths()) {
            autoChooser.addOption(command.getName(), command);
        }
        SmartDashboard.putData(autoChooser);
    }

    private String autoName = "";
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.updateSmartDashboard();

        //Listen for a change in the autonomous chooser so that we can reset the odometry before the match starts
        // This is not necessary but will help since we can see the change on the dashboard before the match starts
        NamedAuto auto = autoChooser.getSelected();
        if (auto != null && !auto.getName().equals(autoName)) {
            autoName = auto.getName();
            //If the pose is valid, reset the odometry with it
            if (auto.getStartPose() != null && auto.getStartRotation() != null) {
                DriverStation.reportError(auto.getStartPose().toString(), false);
                robotContainer.getDrivetrain().resetOdometry(auto.getStartRotation(), auto.getStartPose());
            }else { //Else, reset the odometry to 0 degrees at 0,0
                DriverStation.reportError("0 StartPose", false);
                robotContainer.getDrivetrain().resetOdometry(Rotation2d.fromDegrees(0), new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
            }
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = autoChooser.getSelected().getCommand();

        // Schedule the autonomous command if there is one
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}