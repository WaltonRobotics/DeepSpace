package frc.robot.robot;

import frc.robot.subsystem.SubsystemLimits;
import org.waltonrobotics.util.RobotConfig;

public abstract class LimitedRobot extends RobotConfig {
    public LimitedRobot(String robotName) {
        super(robotName);
    }

    abstract SubsystemLimits getSubsystemLimits();

}
