package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.CargoIntaker;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class CargoHandlingTransition implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.CARGO_HANDLING);
        CargoIntaker.getInstance().flipOutClawSystem();
    }

    @Override
    public State periodic() {
        return null;
    }

    @Override
    public void finish() {


    }
}
