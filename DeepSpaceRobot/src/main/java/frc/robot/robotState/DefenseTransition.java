package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.CargoIntaker;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.HatchIntaker;

public class DefenseTransition implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.DEFENSE);
        Elevator.getInstance().resetElevator();
        HatchIntaker.getHinstance().flipInHatchIntake();
        CargoIntaker.getInstance().flipInClawSystem();
    }

    @Override
    public State periodic() {
        return null;
    }

    @Override
    public void finish() {

    }
}
