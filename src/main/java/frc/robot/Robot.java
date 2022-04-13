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
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private RobotContainer kRobotContainer;

    @Override
    public void robotInit() {
        kRobotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        kRobotContainer.updateSmartDashboard();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }
}