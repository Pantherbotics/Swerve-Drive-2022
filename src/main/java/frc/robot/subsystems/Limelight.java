package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.util.Target;

@SuppressWarnings("unused")
public class Limelight {
    private final NetworkTable limelight;
    private final NetworkTableEntry limelightTX;
    private final NetworkTableEntry limelightTY;
    private final NetworkTableEntry limelightTA;

    private final Target currentTarget;

    public Limelight() {
        this.currentTarget = new Target();

        this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
        this.limelightTX = this.limelight.getEntry("tx");
        this.limelightTY = this.limelight.getEntry("ty");
        this.limelightTA = this.limelight.getEntry("ta");


        Notifier updateLoop = new Notifier(this::update);

        updateLoop.startPeriodic(10f / 1000f);
        setLights(1);
    }

    private void update()
    {
        this.currentTarget.setPitch(this.limelightTY.getDouble(0));
        this.currentTarget.setYaw(this.limelightTX.getDouble(0));
        this.currentTarget.setArea(this.limelightTA.getDouble(0));
    }

    public Target getTarget()
    {
        return this.currentTarget;
    }

    public void setLights(int state) {
        this.limelight.getEntry("ledMode").setNumber(state);
    }

    public boolean hasTarget() {
        return this.limelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getDistance() {
        /*
        double goalHeight = 8*12 + 8;
        double camHeight = 36.625;
        double camAngle = 35;

        double pitch = currentTarget.pitch + camAngle;
        return ((goalHeight-camHeight) / Math.tan(Math.toRadians(pitch)));
        */
        return 0;
    }
}
