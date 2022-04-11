package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Util.DriveCommand;
import frc.robot.Util.ModuleLocation;
import frc.robot.commands.Drive;
import frc.robot.SwerveModule;

public class Drivetrain extends Subsystem {
  private SwerveModule moduleFrontLeft = new SwerveModule(Constants.kFrontLeftSteerID, Constants.kFrontLeftDriveID, true, Constants.kFrontLeftOffset, Constants.kSwerveP, Constants.kSwerveI, Constants.kSwerveD);
  private SwerveModule moduleFrontRight = new SwerveModule(Constants.kFrontRightSteerID, Constants.kFrontRightDriveID, true, Constants.kFrontRightOffset, Constants.kSwerveP, Constants.kSwerveI, Constants.kSwerveD);
  private SwerveModule moduleBackLeft = new SwerveModule(Constants.kBackLeftSteerID, Constants.kBackLeftDriveID, true, Constants.kBackLeftOffset, Constants.kSwerveP, Constants.kSwerveI, Constants.kSwerveD);
  private SwerveModule moduleBackRight = new SwerveModule(Constants.kBackRightSteerID, Constants.kBackRightDriveID, true, Constants.kBackRightOffset, Constants.kSwerveP, Constants.kSwerveI, Constants.kSwerveD);
  public Drivetrain(){
    moduleFrontLeft.configEncValues(47, 855, 156, 978);
    moduleFrontRight.configEncValues(44, 853, 156, 978);
    moduleBackLeft.configEncValues(45, 868, 156, 978);
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Drive());
  }
  
  public SwerveModule getModule(ModuleLocation location){
    switch(location){
      case FRONT_LEFT:
        return moduleFrontLeft;
      case FRONT_RIGHT:
        return moduleFrontRight;
      case BACK_LEFT:
        return moduleBackLeft;
      case BACK_RIGHT:
        return moduleBackRight;
      default:
        return null;
    }
  }

  public void setModule(ModuleLocation loc, double degrees, double power){
    switch(loc){
      case FRONT_LEFT:
        moduleFrontLeft.set(degrees, power);
        break;
      case FRONT_RIGHT:
        moduleFrontRight.set(degrees, power);
        break;
      case BACK_LEFT:
        moduleBackLeft.set(degrees, power);
        break;
      case BACK_RIGHT:
        moduleBackRight.set(degrees, power);
        break;
    }
  }

  public void setModule(ModuleLocation loc, DriveCommand command){
    setModule(loc, command.getDegrees(), command.getSpeed());
  }

  public void setAllAngle(double degrees){
    moduleFrontLeft.setSteeringDegrees(degrees);
    moduleFrontRight.setSteeringDegrees(degrees);
    moduleBackLeft.setSteeringDegrees(degrees);
    moduleBackRight.setSteeringDegrees(degrees);
  }

  public void setAllPower(double power){
    moduleFrontLeft.setDrivePower(power);
    moduleFrontRight.setDrivePower(power);
    moduleBackLeft.setDrivePower(power);
    moduleBackRight.setDrivePower(power);
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Front Left Error", moduleFrontLeft.getError());
    SmartDashboard.putNumber("Front Right Error", moduleFrontRight.getError());
    SmartDashboard.putNumber("Back Left Error", moduleBackLeft.getError());
    SmartDashboard.putNumber("Back Right Error", moduleBackRight.getError());

    SmartDashboard.putNumber("Front Left Setpoint", moduleFrontLeft.getSetpointDegrees());
    SmartDashboard.putNumber("Front Right Setpoint", moduleFrontRight.getSetpointDegrees());
    SmartDashboard.putNumber("Back Left Setpoint", moduleBackLeft.getSetpointDegrees());
    SmartDashboard.putNumber("Back Right Setpoint", moduleBackRight.getSetpointDegrees());

    SmartDashboard.putNumber("Front Left Position", moduleFrontLeft.getSteeringDegrees());
    SmartDashboard.putNumber("Front Right Position", moduleFrontRight.getSteeringDegrees());
    SmartDashboard.putNumber("Back Left Position", moduleBackLeft.getSteeringDegrees());
    SmartDashboard.putNumber("Back Right Position", moduleBackRight.getSteeringDegrees());

    SmartDashboard.putNumber("Front Left Raw Position", moduleFrontLeft.getRawSteeringEncoder());
    SmartDashboard.putNumber("Front Right Raw Position", moduleFrontRight.getRawSteeringEncoder());
    SmartDashboard.putNumber("Back Left Raw Position", moduleBackLeft.getRawSteeringEncoder());
    SmartDashboard.putNumber("Back Right Raw Position", moduleBackRight.getRawSteeringEncoder());

    SmartDashboard.putNumber("Front Left Speed", moduleFrontLeft.getSpeed());
    SmartDashboard.putNumber("Front Right Speed", moduleFrontRight.getSpeed());
    SmartDashboard.putNumber("Back Left Speed", moduleBackLeft.getSpeed());
    SmartDashboard.putNumber("Back Right Speed", moduleBackRight.getSpeed());
  }

  public double getGyro(){
    return 0;
  }
}
