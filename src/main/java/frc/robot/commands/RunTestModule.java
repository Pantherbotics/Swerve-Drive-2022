package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.SwerveModule;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
public class RunTestModule extends CommandBase {
    private final SwerveModule module;
    private final List<Double> angles = new ArrayList<>();
    public RunTestModule(SwerveModule module) {
        this.module = module;

        //Add angles from 0 to 360 and then from 360 to 0 at intervals of 10 degrees, each time adding 10 degrees
        //Predicted Result: The wheel should turn clockwise fully stepping 10 degrees, then come back to 0 the same way
        //  It will then repeat this process stepping more each time, eventually it should go from 0 to more than 180
        //  and it should go counterclockwise to prove that the shortest path is taken
        for (int i = 1; i <= 36; i++) {
            for (double x = 0; x <= 360; x += 10*i) {
                angles.add(x);
            }
            for (double x = 360; x >= 0; x -= 10*i) {
                angles.add(x);
            }
        }
    }

    int index = 0;
    int successes = 0;
    int failures = 0;
    boolean sent = false;
    @Override
    public void execute() {
        if (!sent) {
            module.update(0, angles.get(index));
            sent = true; successes = 0; failures = 0;
        }
        if (module.isAtTarget()) {
            successes++;
        }else {
            failures++;
        }
        if (failures >= 5) {
            failures = 0; successes = 0;
        }
        if (successes >= 5 && (double)successes / (double)(successes + failures) >= 0.8) {
            index++;
            sent = false;
        }
    }
}
