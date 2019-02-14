package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class DefenseTransition implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.DEFENSE);
        Robot.godSubsystem.resetElevator();
        Robot.godSubsystem.flipInHatchIntake();
        Robot.godSubsystem.flipInClawSystem();
    }

    @Override
    public State periodic() {
        return null;
    }

    @Override
    public void finish() {

    }
}
