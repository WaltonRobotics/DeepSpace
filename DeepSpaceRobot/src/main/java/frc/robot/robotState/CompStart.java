package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class CompStart implements State {

    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.ROBOT_SWITCHED_ON);


    }

    @Override
    public State periodic() {
        ElevatorCargoHatchSubsystem.ActiveState currentActiveState = Robot.godSubsystem.getCurrentActiveState();

        if (currentActiveState == ElevatorCargoHatchSubsystem.ActiveState.CARGO_HANDLING)
            return new CargoHandlingTransition();
        else if ((currentActiveState == ElevatorCargoHatchSubsystem.ActiveState.DEFENSE))
            return new DefenseTransition();
        else
            return new HatchHandlingTransition();
    }

    @Override
    public void finish() {

    }
}
