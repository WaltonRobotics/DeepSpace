package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.*;

import static frc.robot.Robot.currentRobot;

/**
 * @author Marius Juston
 **/
public class SetCompStartHatch implements State {

    private final Hatch hatch = Robot.godSubsystem.getHatch();
    private final Cargo cargo = Robot.godSubsystem.getCargo();
    private final Elevator elevator = Robot.godSubsystem.getElevator();

    @Override
    public void initialize() {
        hatch.setLimits(HatchPosition.HATCH_START);
        cargo.setLimits(CargoPosition.SAFE);
        elevator.setElevatorLevel(ElevatorLevel.HATCH_BASE);
    }

    @Override
    public State periodic() {
        if (!Robot.godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (currentRobot.getTarget(ElevatorLevel.HATCH_BASE).isClose(elevator.getElevatorHeight(), 250)) {
            hatch.setCurrentTarget(HatchPosition.HATCH_START);
            cargo.setCurrentTarget(CargoPosition.SAFE);
        }

        if (Robot.godSubsystem.cargoModeRising()) {
            return new CargoHandlingTransition();
        }
        if (Robot.godSubsystem.defenceModeRising()) {
            return new DefenseTransition();
        }

        if (Robot.godSubsystem.hatchModeRising()) {
            return new HatchHandlingTransition();
        }

        return this;
    }

    @Override
    public void finish() {

    }

    @Override
    public String toString() {
        return "SetCompStartHatch{" +
                "hatch=" + hatch +
                ", cargo=" + cargo +
                ", elevator=" + elevator +
                '}';
    }
}
