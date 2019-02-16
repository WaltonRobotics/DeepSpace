package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;

public class DefenseTransition implements State {

    private ActiveState lastState;

    @Override
    public void initialize() {
        if(Robot.godSubsystem.getCurrentActiveState() == ElevatorCargoHatchSubsystem.ActiveState.CARGO_HANDLING){
            lastState = ActiveState.CARGO_HANDLING;
        }
        if(Robot.godSubsystem.getCurrentActiveState() == ElevatorCargoHatchSubsystem.ActiveState.HATCH_HANDLING){
            lastState = ActiveState.HATCH_HANDLING;
        }
        if(Robot.godSubsystem.getCurrentActiveState() == ActiveState.DEFENSE){
            lastState = ActiveState.DEFENSE;
        }
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.DEFENSE);
        Robot.godSubsystem.getElevator().resetElevator();
        if(lastState == ActiveState.HATCH_HANDLING) {
            if (!ElevatorCargoHatchSubsystem.HatchPosition.SAFE.inRange(Robot.godSubsystem.getHatch().getAngle())) {
                Robot.godSubsystem.flipInHatchIntake();
            }
        }
        if(lastState == ActiveState.CARGO_HANDLING){
            if (!ElevatorCargoHatchSubsystem.HatchPosition.SAFE.inRange(Robot.godSubsystem.getHatch().getAngle())) {
                Robot.godSubsystem.flipOutHatchIntake();
            }
            if (ElevatorCargoHatchSubsystem.CargoPosition.SAFE.inRange(Robot.godSubsystem.getCargo().getAngle())) {
                Robot.godSubsystem.getCargo().flipInClawSystem();
            }
        }
    }

    @Override
    public State periodic() {
        return this;
    }

    @Override
    public void finish() {

    }
}
