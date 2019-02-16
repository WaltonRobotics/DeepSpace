package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class HatchHandlingTransition implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.HATCH_HANDLING);
        Robot.godSubsystem.getCargo().setClawTarget(CargoPosition.SAFE);
        Robot.godSubsystem.getHatch().setHatchTarget(HatchPosition.DEPLOY);
    }

    @Override
    public State periodic() {
        int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
        int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

        if(CargoPosition.SAFE.isClose(cargoAngle) && HatchPosition.DEPLOY.isClose(hatchAngle))
            return new HatchHandling();

        return this;
    }

    @Override
    public void finish() {

    }
}
