package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class HatchHandlingTransition implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.HATCH_HANDLING);
        Robot.godSubsystem.flipOutHatchIntake();
    }

    @Override
    public State periodic() {
        return this;
    }

    @Override
    public void finish() {

    }
}
