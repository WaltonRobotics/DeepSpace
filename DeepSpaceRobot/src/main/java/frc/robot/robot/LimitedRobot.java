package frc.robot.robot;

import frc.robot.subsystem.SubsystemLimits;
import frc.robot.subsystem.SusystemTargets;
import org.waltonrobotics.util.RobotConfig;

public abstract class LimitedRobot extends RobotConfig {
    public LimitedRobot(String robotName) {
        super(robotName);
    }

    abstract SubsystemLimits getCargoSubsystemLimits();
    abstract SubsystemLimits getHatchSubsystemLimits();
    abstract SubsystemLimits getElevatorSubsystemLimits();
    abstract SusystemTargets getTargets();

    public double getTarget(String target){
        return getTargets().getTargets().get(target);
    }

}
