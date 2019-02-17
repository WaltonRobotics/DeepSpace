package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;

public class TakeControl implements State {

    @Override
    public void initialize() {
        Robot.godSubsystem.resetElevator();

        /* Reset other components as well. */
    }

    @Override
    public State periodic() {
        return null;
    }

    @Override
    public void finish() {

    }

}
