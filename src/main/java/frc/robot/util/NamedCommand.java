package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A class that wraps a Command with a name
 */
public class NamedCommand {
    private final String name;
    private final Command comm;

    public NamedCommand(String name, Command comm) {
        this.name = name;
        this.comm = comm;
    }

    public String getName() {
        return name;
    }

    public Command getCommand() {
        return comm;
    }
}