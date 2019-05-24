package frc.robot.robotState;

import frc.robot.command.teleop.Drive;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.*;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.godSubsystem;

/**
 * @author Marius Juston
 **/
public class AutoClimbStage3 implements State {

    private final Climber climber = godSubsystem.getClimber();
    private double timeout;

    @Override
    public void initialize() {
        Drive.setIsEnabled(false);
        timeout = godSubsystem.getCurrentTime() + 1000L;

        climber.setTimer(3000, -1.0);
        drivetrain.setSpeeds(0.5, 0.5);

        godSubsystem.setCurrentActiveState(ActiveState.DEFENSE);
        godSubsystem.getHatch().setCurrentTarget(HatchPosition.DEFENSE);
        godSubsystem.getHatch().setLimits(HatchPosition.SAFE);
        godSubsystem.getCargo().setCurrentTarget(CargoPosition.SAFE);
        godSubsystem.getCargo().setLimits(CargoPosition.SAFE);

        godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
        godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);
        godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);
    }

    @Override
    public State periodic() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        if (godSubsystem.isMasterOverride()) {
            return new ClimbHandlingTransition();
        }

        if (godSubsystem.getCurrentTime() > timeout) {
            return new DefenseTransition();
        }

        return this;
    }

    @Override
    public void finish() {
        drivetrain.setSpeeds(0, 0);
        Drive.setIsEnabled(true);
    }

    @Override
    public String toString() {
        return "AutoClimbStage3{" +
                "climber=" + climber +
                ", timeout=" + timeout +
                '}';
    }
}
