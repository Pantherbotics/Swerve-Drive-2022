package frc.robot;

@SuppressWarnings("unused")
public class Constants{
    //Motors
    public static int kFrontLeftDriveID = 1;
    public static int kFrontLeftSteerID = 2;
    public static int kFrontRightDriveID = 3;
    public static int kFrontRightSteerID = 4;
    public static int kBackLeftDriveID = 5;
    public static int kBackLeftSteerID = 6;
    public static int kBackRightDriveID = 7;
    public static int kBackRightSteerID = 8;
    
    //Module-Specific
    public static double kFrontLeftOffset = 31;//48;
    public static double kFrontRightOffset = -46;//-54;
    public static double kBackLeftOffset = -98;//-94;
    public static double kBackRightOffset = 153;

    //Buttons/Joystick
    public static int kJoyStick = 0;
    public static int kJoystickLeft = 1;
    public static int kToggleDrive = 4;
    public static int kSetMotorSpeed = 0;
    public static double kLeftYOffset = 0.2;

    //Physical Constants
    public static double kWheelbase = 18.0;
    public static double kTrackwidth = 18.0;
    public static double kTurnRadius = Math.hypot(kWheelbase, kTrackwidth);

    //PID
    public static double kSwerveP = 4.8;
    public static double kSwerveI = 0.0;
    public static double kSwerveD = 0.0;

    public static int kPeakCurrentDuration = 5;
    public static int kPeakCurrentLimit = 5;
    public static int kSustainedCurrentLimit = 15;
    public static int kPIDLoopIdx = 0;
    public static int kTimeoutMs = 30;
}

