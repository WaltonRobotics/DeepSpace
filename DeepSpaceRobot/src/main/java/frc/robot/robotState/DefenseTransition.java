package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

public class DefenseTransition implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.DEFENSE);
        Robot.godSubsystem.getElevator().resetElevator();
    }

    @Override
    public State periodic() {
        //not exactly sure how to do this but the logic is there
        while (RobotMap.hatchPotentiometer.get() < 100){
            Robot.godSubsystem.flipOutHatchIntake();
        }
        while (RobotMap.hatchPotentiometer.get() > 100){
            Robot.godSubsystem.flipInHatchIntake();
        }
        while (RobotMap.cargoPotentiometer.get() > 100){
            Robot.godSubsystem.flipInClawSystem();
        }
        return null;
    }

    @Override
    public void finish() {

    }
}
