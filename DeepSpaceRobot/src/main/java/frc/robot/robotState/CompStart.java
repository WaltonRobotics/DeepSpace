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
        return null;
    }

    @Override
    public void finish() {

    }
}
