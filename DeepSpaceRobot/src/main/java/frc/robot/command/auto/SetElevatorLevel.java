package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

import static frc.robot.Robot.godSubsystem;

public class SetElevatorLevel extends Command {

    private int counter;

    private ElevatorCargoHatchSubsystem.ElevatorLevel elevatorLevel;

    public SetElevatorLevel(ElevatorCargoHatchSubsystem.ElevatorLevel level) {
        elevatorLevel = level;
        counter = 0;
    }

    @Override
    protected void initialize() {
        godSubsystem.getElevator().setElevatorLevel(elevatorLevel);
        counter++;
    }

    @Override
    protected boolean isFinished() {
        return counter >= 1;
    }
}
