package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class Disabled implements State {
    @Override
    public void initialize() {



    }

    @Override
    public State periodic() {
        return this;
    }

    @Override
    public void finish() {

    }
}
