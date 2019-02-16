package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class CargoHandlingTransition implements State {
    @Override
    public void initialize() {

        // set limits for play mode
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.CARGO_HANDLING);
        Robot.godSubsystem.getHatch().setClawTarget(HatchPosition.SAFE);
        Robot.godSubsystem.getCargo().setClawTarget(CargoPosition.DEPLOY);

    }

    @Override
    public State periodic() {

        int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
        int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

        if(CargoPosition.DEPLOY.isClose(cargoAngle) && HatchPosition.SAFE.isClose(hatchAngle))
            return new CargoHandling();
        else
            return this;
    }

    @Override
    public void finish() {

    }
}
