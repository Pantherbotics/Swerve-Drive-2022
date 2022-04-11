package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Util.DriveCommand;
import frc.robot.Util.WrappedTalonSRX;

@SuppressWarnings("unused")
public class SwerveModule{
    //private AnalogInput SteeringAnalog = new AnalogInput(0);
    private final WrappedTalonSRX mDrive, mSteering;
    private volatile double sumError, errorChange, lastError, currentError, pidOutput;
    private final boolean isReversed;
    private double setpoint;
    private final double offset;

    private double lastAngle;

    private static final double dt = 0.02;  //this is how fast we run our PID loop.
    private int kPositiveRotationMin = 45;  //we measured this
    private int kPositiveRotationMax = 870;  //and this

    private int kNegativeRotationMin = 156;  //we measured this
    private int kNegativeRotationMax = 978;  //and this
    
    /**
     * 
     * @param kSteeringID   the ID of the steering motor
     * @param kDriveID      the ID of the drive motor
     * @param isReversed    if the module is physically reversed on the robot
     * @param kP            the steering kP gain
     * @param kI            the steering kI gain
     * @param kD            the steering kD gain
     */
    public SwerveModule(int kSteeringID, int kDriveID, boolean isReversed, double offset, double kP, double kI, double kD){
        mDrive = new WrappedTalonSRX(kDriveID);
        mSteering = new WrappedTalonSRX(kSteeringID);
        this.offset = offset;

        lastAngle = 0;

        //reset the Talons before use
        mDrive.reset();
        mSteering.reset();

        //Configure steering Talon SRX
        mSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        mSteering.configOpenloopRamp(0, Constants.kTimeoutMs);      //this is what we were missing!
        mSteering.configPeakCurrentDuration(Constants.kPeakCurrentDuration, Constants.kTimeoutMs);
        mSteering.configPeakCurrentLimit(Constants.kPeakCurrentLimit, Constants.kTimeoutMs);
        mSteering.configContinuousCurrentLimit(Constants.kSustainedCurrentLimit, Constants.kTimeoutMs);
        mSteering.enableCurrentLimit(true);
        mSteering.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10, 0);
        mSteering.setInverted(true);
        mSteering.setSensorPhase(true);

        //Configure drive Talon SRX
        mDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /*
        sumError = 0;
        lastError = getModifiedError();
        currentError = lastError;
        */

        //update the current error to the most recent one
        /*
            sumError += currentError * dt;
            errorChange = (currentError-lastError)/dt;
*/
        //+ kI * sumError + kD * errorChange; //you guys know this, or at least you better...
        //lastError = currentError;   //update the last error to be the current error
        //A notifier is a thread. Basically think of a thread as something running in the background.
        Notifier pidLoop = new Notifier(() -> {
            currentError = getModifiedError();  //update the current error to the most recent one
            /*
            sumError += currentError * dt;
            errorChange = (currentError-lastError)/dt;
*/
            pidOutput = kP * currentError; //+ kI * sumError + kD * errorChange; //you guys know this, or at least you better...
            mSteering.set(ControlMode.PercentOutput, pidOutput);
            //lastError = currentError;   //update the last error to be the current error
        });


        this.isReversed = isReversed;

        pidLoop.startPeriodic(dt);
    }

    /**@return  the velocity of the wheel, measured in ticks/100ms
     * 
     */
    public double getDriveEncoder(){
        return (mDrive.getSelectedSensorVelocity(0));
    }

    /**
     * @return  the angle of the wheel, where angle is an element of [-pi, pi]
     */

    public double getSteeringDegrees(){
        double steeringPosition = mSteering.getSelectedSensorPosition(Constants.kPIDLoopIdx);

        if(steeringPosition >= 0){
            return normalizeEncoder(kPositiveRotationMin, kPositiveRotationMax, steeringPosition)-180;
        }
        else
            return (360-normalizeEncoder(kNegativeRotationMin, kNegativeRotationMax, steeringPosition))-180;



    }

    public double getSteeringDegreesCompensated(){
        return getSteeringDegrees() - offset;
    }
    /**
     * 
     * @return  the closed-loop PID output, calculated by PID loop
     */
    public double getSteeringOutput(){
        return pidOutput;
    }


    /**
     * 
     * @return  the unbounded steering error, in radians
     */
    public double getError(){
        return setpoint - getSteeringDegrees();
    }

    /**
     * 
     * @return  the steering error bounded to [-pi, pi]
     */
    public double getModifiedError(){
        return (boundHalfDegrees(getError()))/180;
    }

    /**
     * 
     * @param power the power of the wheel, where power is [-1.0, 1.0]
     */
    public void setDrivePower(double power){
        if(isReversed)
            mDrive.set(ControlMode.PercentOutput, -power);
        else
            mDrive.set(ControlMode.PercentOutput, power);
    }

    /**
     * 
     * @param deg   the angle to set the wheel to, in degrees
     */
    public void setSteeringDegrees(double deg){
        setpoint = boundHalfDegrees(deg + offset);
    }

    /**
     * 
     * @return  returns the setpoint of the steering in degrees
     */
    public double getSetpointDegrees(){
        return setpoint;
    }

    /**
     *
     * @param encPos    the encoder input to be normalized
     * @param minVal    the minimum MEASURED ABSOLUTE value of the encoder
     * @param maxVal    the maximum MEASURED ABSOLUTE value of the encoder
     * @return          the encoder input normalized to [0, 1023]
     * */
    private double normalizeEncoder(int minVal, int maxVal, double encPos){
        return ((Math.abs(encPos) % 1023) - minVal) * Math.abs((360.0/(maxVal-minVal)));
    }

    public void setSteeringPower(double x){
        mSteering.set(ControlMode.PercentOutput, x);
    }

    public void set(double degrees, double power){
        double supplement = degrees > 0 ? degrees - 180 : 180 + degrees;

        if(Math.abs(supplement-lastAngle) <= 90){
            setSteeringDegrees(supplement);
            setDrivePower(-power);
            lastAngle = supplement;
        }
        else {
            setSteeringDegrees(degrees);
            setDrivePower(power);
            lastAngle = degrees;
        }
    }

    public void set(DriveCommand command){
        set(command.getDegrees(), command.getSpeed());
    }

    public void configEncValues(int posMin, int posMax, int negMin, int negMax){
        kPositiveRotationMin = posMin;
        kPositiveRotationMax = posMax;

        kNegativeRotationMin = negMin;
        kNegativeRotationMax = negMax;
    }

    public double getRawSteeringEncoder(){
        return mSteering.getSelectedSensorPosition(0);
    }

    public double getSpeed(){
        return mDrive.getSelectedSensorVelocity(0);
    }

    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }
    
}
