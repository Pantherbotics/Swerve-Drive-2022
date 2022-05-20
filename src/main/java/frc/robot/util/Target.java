package frc.robot.util;

import lombok.Getter;
import lombok.Setter;

@SuppressWarnings("unused")
public class Target {
    @Getter @Setter private double  pitch    = 0;
    @Getter @Setter private double  yaw      = 0;
    @Getter @Setter private double  area     = 0;
}