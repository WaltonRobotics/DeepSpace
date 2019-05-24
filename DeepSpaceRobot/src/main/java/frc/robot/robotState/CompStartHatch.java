package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Hatch;

public class CompStartHatch implements State {

    private final Hatch hatch = Robot.godSubsystem.getHatch();
    private final Cargo cargo = Robot.godSubsystem.getCargo();
    private double timeout;

    @Override
    public void initialize() {
        cargo.setLimits(CargoPosition.SAFE);

        hatch.setIntake(true);
        timeout = Robot.godSubsystem.getCurrentTime() + 500L;
    }

    @Override
    public State periodic() {

        if (!Robot.godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (Robot.godSubsystem.getCurrentTime() >= timeout) {
            return new HatchHandlingTransition();
        }

        return this;
    }

    @Override
    public void finish() {

    }

    @Override
    public String toString() {
        return "CompStartHatch{" +
                "hatch=" + hatch +
                ", cargo=" + cargo +
                ", timeout=" + timeout +
                '}';
    }
}
