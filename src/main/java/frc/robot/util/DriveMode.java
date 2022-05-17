package frc.robot.util;

@SuppressWarnings("unused")
public enum DriveMode {
    /**
     * Field Oriented Swerve
     */
    FO_SWERVE,

    /**
     * Regular Swerve modes W/O any Gyro Alignment
     */
    SWERVE,
    CAR,
    BOAT,
    WESTCOAST,
    TANK;

    public String getName() {
        if (this == FO_SWERVE) {
            return "Field Oriented Swerve";
        } else if (this == SWERVE) {
            return "Swerve";
        } else if (this == CAR) {
            return "Car";
        } else if (this == BOAT) {
            return "Boat";
        }else if (this == WESTCOAST) {
            return "West Coast";
        }else if (this == TANK) {
            return "Tank";
        }
        return this.name();
    }
}

//Swerve is the primary/favored control mode
//Other modes are not implemented yet. (As of 4/27/2022)