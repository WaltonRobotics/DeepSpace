package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class Defense implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.DEFENSE);
    }

    @Override
    public State periodic() {

        if(!Robot.godSubsystem.isEnabled()){
            return new Disabled();
        }
        if(Robot.godSubsystem.cargoModeRising()){
            return new CargoHandlingTransition();
        }
        if(Robot.godSubsystem.hatchModeRising()){
            return new HatchHandlingTransition();
        }
        return this;
    }

    @Override
    public void finish() {

    }
}
