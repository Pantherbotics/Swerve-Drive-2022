/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.DriveCommand;
import frc.robot.util.ModuleLocation;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.oi;

public class Drive extends Command {
  
  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double leftX = oi.getLeftXAxis();
    double leftY = oi.getLeftYAxis();
    double rightX = oi.getRightXAxis();
    double heading = Math.toRadians(Robot.drivetrain.getGyro());

    /*
    double temp = leftY*Math.cos(heading) + leftX*Math.sin(heading);
    leftX = -leftY*Math.sin(heading) + leftX*Math.cos(heading);
    leftY = temp;
    */

    double A = leftX - rightX * (Constants.kWheelbase/Constants.kTrackwidth);
    double B = leftX + rightX * (Constants.kWheelbase/Constants.kTrackwidth);
    double C = leftY - rightX * (Constants.kTrackwidth/Constants.kWheelbase);
    double D = leftY + rightX * (Constants.kTrackwidth/Constants.kWheelbase);

    double frontLeftSpeed = Math.hypot(B, C);
    double frontRightSpeed = Math.hypot(B, D);
    double backLeftSpeed = Math.hypot(A, C);
    double backRightSpeed = Math.hypot(A, D);
    
    double max = frontLeftSpeed;

    if(frontRightSpeed > max)
      max = frontRightSpeed;
    else if(backLeftSpeed > max)
      max = backLeftSpeed;
    else if(backRightSpeed > max)
      max = backRightSpeed;
    if(max > 1){
      frontLeftSpeed /= max;
      frontRightSpeed /= max;
      backLeftSpeed /= max;
      backRightSpeed /= max;
    }

    DriveCommand frontLeftCommand = new DriveCommand((Math.atan2(B, C) * (180.0/Math.PI)), frontLeftSpeed);
    DriveCommand frontRightCommand = new DriveCommand((Math.atan2(B, D) * (180.0/Math.PI)), frontRightSpeed);
    DriveCommand backLeftCommand = new DriveCommand((Math.atan2(A, C) * (180.0/Math.PI) ), backLeftSpeed);
    DriveCommand backRightCommand = new DriveCommand((Math.atan2(A, D) * (180.0/Math.PI)), backRightSpeed);

    drivetrain.setModule(ModuleLocation.FRONT_LEFT, frontLeftCommand);
    drivetrain.setModule(ModuleLocation.FRONT_RIGHT, frontRightCommand);
    drivetrain.setModule(ModuleLocation.BACK_LEFT, backLeftCommand);
    drivetrain.setModule(ModuleLocation.BACK_RIGHT, backRightCommand);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
