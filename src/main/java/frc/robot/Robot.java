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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.NamedCommand;

public class Robot extends TimedRobot {
    public final SendableChooser<NamedCommand> autoChooser = new SendableChooser<>();
    public final SendableChooser<Double> speedChooser = new SendableChooser<>();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        speedChooser.setDefaultOption("Normal (Fast 100%)", 1.0);
        speedChooser.addOption("Safe (Slow 25%)", 0.25);
        SmartDashboard.putData(speedChooser);

        robotContainer = new RobotContainer(this);

        autoChooser.setDefaultOption("None", null);
        for (NamedCommand command : robotContainer.autoPaths.paths) {
            autoChooser.addOption(command.getName(), command);
        }
        SmartDashboard.putData(autoChooser);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.updateSmartDashboard();
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

        // schedule the autonomous command (example)
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