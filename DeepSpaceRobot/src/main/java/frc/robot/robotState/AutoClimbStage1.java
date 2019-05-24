package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.command.teleop.Drive;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.*;

import static frc.robot.Robot.*;
import static frc.robot.RobotMap.elevatorMotor;

/**
 * @author Marius Juston
 **/
public class AutoClimbStage1 implements State {

    private static final int velocity = 400;

    @Override
    public void initialize() {
        godSubsystem.setCurrentActiveState(ActiveState.CARGO_HANDLING);

        godSubsystem.getHatch().setCurrentTarget(HatchPosition.SAFE);
        godSubsystem.getCargo().setCurrentTarget(CargoPosition.CLIMB);
        godSubsystem.getCargo().setLimits(CargoPosition.CLIMB);

        godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
        godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);
        godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);

        godSubsystem.getClimber().setClimberControlMode(ClimberControlMode.MANUAL);
        elevatorMotor.configMotionCruiseVelocity(velocity);

        Drive.setIsEnabled(false);
    }

    @Override
    public State periodic() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (godSubsystem.isMasterOverride()) {
            return new ClimbHandlingTransition();
        }

        drivetrain.setSpeeds(1.0, 1.0);

        int elevatorHeight = godSubsystem.getElevator().getElevatorHeight();
        int cargoAngle = godSubsystem.getCargo().getAngle();

        return (((Robot.currentRobot.getTarget(CargoPosition.CLIMB)
                .isClose(cargoAngle, 50) &&
                Robot.currentRobot.getTarget(ElevatorLevel.HATCH_BASE)
                        .isClose(elevatorHeight, 250)) && godSubsystem.autoClimbRising()) || Robot.godSubsystem.isMasterOverride())
                ?
                new AutoClimbStage2() : this;
    }

    @Override
    public void finish() {
        Drive.setIsEnabled(true);
        elevatorMotor.configMotionCruiseVelocity(currentRobot.getElevatorSubsystemLimits().getMotionCruiseVelocity());
    }
}