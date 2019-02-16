package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;

public class TakeControl implements State {

    @Override
    public void initialize() {

    }

    @Override
    public State periodic() {
        if(!Robot.godSubsystem.isEnabled()){
            return new Disabled();
        }

        return this;
    }

    @Override
    public void finish() {

    }
}
