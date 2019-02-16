package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class DefenseTransition implements State {

    private ActiveState lastState;

    @Override
    public void initialize() {
        switch (Robot.godSubsystem.getCurrentActiveState()){
            case HATCH_HANDLING:
                lastState = ActiveState.HATCH_HANDLING;
            case CARGO_HANDLING:
                lastState = ActiveState.CARGO_HANDLING;
            case DEFENSE:
                lastState = ActiveState.DEFENSE;
        }
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.DEFENSE);
        Robot.godSubsystem.getElevator().resetElevator();
        switch (lastState){
            case CARGO_HANDLING:
                Robot.godSubsystem.getHatch().setHatchTarget(HatchPosition.SAFE);
                Robot.godSubsystem.getCargo().setClawTarget(CargoPosition.SAFE);
            case HATCH_HANDLING:
                Robot.godSubsystem.getHatch().setHatchTarget(HatchPosition.SAFE);
        }
    }

    @Override
    public State periodic() {

        if(!Robot.godSubsystem.isEnabled()){
            return new Disabled();
        }

        int cargoAngle = Robot.godSubsystem.getCargo().getAngle();
        int hatchAngle = Robot.godSubsystem.getHatch().getAngle();

        if(CargoPosition.SAFE.isClose(cargoAngle) && HatchPosition.SAFE.isClose(hatchAngle)){
            return new Defense();
        }
        return this;
    }

    @Override
    public void finish() {

    }
}
